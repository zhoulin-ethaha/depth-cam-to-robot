import asyncio
import os
import signal
import sys
import threading
import webbrowser

# Force UTF-8 console output so Unicode in log messages (→, ×, …) never crashes
# the program on Windows consoles that default to a legacy code page (cp1252).
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass

import math

from config import (
    HTTP_HOST, HTTP_PORT, CONTOUR_MIN_PIXELS,
    DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS, DEPTH_AVERAGE_FRAMES, SURFACE_DIR,
    DRAW_Z, DRAW_SPEED, TRAVEL_Z, MAX_TCP_SPEED,
    UR_REACH_M, UR_MIN_REACH_M,
)
from camera_thread import DepthCameraThread
from depth_extractor import (
    Crop, DepthGrooveParams, colorize_depth, encode_jpeg, process_depth,
)
from path_extractor import extract_from_edges, pixels_to_robot_coords
from path_executor import PathExecutor
from robot_controller import RobotController
from server import Server
from settings import load_settings, save_settings
from surface import SurfaceModel, SurfacePose
from workspace import WorkspaceConfig

# ── Shared state ──────────────────────────────────────────────────────────────
shared_state: dict = {
    "robot_connected":     False,
    "last_depth_color_jpg": None,    # colorized depth (live view)
    "last_rgb_jpg":        None,     # aligned colour image (live view)
    "last_groove_jpg":     None,     # detected groove skeleton (live preview)
    "last_mask_jpg":       None,     # thick detected-region mask (live preview)
    "last_mask_full_jpg":  None,     # full-frame mask for the projector (gated)
    "projection_clients":  0,        # connected projection windows
    "workspace":           None,     # WorkspaceConfig | None — confirmed workspace
    "pending_workspace":   None,     # WorkspaceConfig | None — loaded from disk
    "ws_points":           {"p0": None, "px": None, "py": None},
    "freedrive":           False,
    "ee":                  [0.0] * 6,
    "phase":               "idle",     # idle|previewing|editing|captured|executing|done|error
    "captured_still":      None,       # (depth_m, valid, rgb) — frozen averaged depth + colour
    "reference_depth":     None,       # baseline depth frame for background subtraction
    "surface_model":       None,       # SurfaceModel | None — target mesh for 3D projection
    "surface_info":        None,       # dict for the browser (name/faces/bbox)
    "surface_pose":        SurfacePose().to_dict(),   # placement in robot base frame
    "surface_offset_mm":   0.0,        # TCP offset along the surface normal
    "surface_mesh_payload": None,      # local-frame vertices/faces for the 3D preview
    "strokes_surface":     False,      # True when current strokes were surface-projected
    "still_dims":          None,       # (width, height) of the captured still
    "strokes":             [],         # robot-space strokes after Generate Path
    "executing":           False,
    "progress":            0.0,
}
state_lock = threading.Lock()

# ── Singletons ────────────────────────────────────────────────────────────────
robot         = RobotController()
camera_thread = DepthCameraThread(shared_state, state_lock)
path_executor = PathExecutor(robot, shared_state, state_lock)


# ── Robot connection callbacks ────────────────────────────────────────────────
async def on_robot_connect(ip: str, ws) -> None:
    loop = asyncio.get_running_loop()
    try:
        await asyncio.wait_for(
            loop.run_in_executor(None, robot.connect, ip),
            timeout=10.0,
        )
        with state_lock:
            shared_state["robot_connected"] = True

        save_settings({"last_ip": ip})

        await server.send_connection_result(
            ws, True, f"Connected to {ip} — load a 3D surface to start."
        )
        print(f"Robot connected: {ip}")

    except asyncio.TimeoutError:
        with state_lock:
            shared_state["robot_connected"] = False
        msg = (
            f"Timeout: no RTDE response from {ip} after 10 s. "
            "Is the robot in Remote Control mode?"
        )
        await server.send_connection_result(ws, False, msg)
        print(msg)
    except Exception as exc:
        with state_lock:
            shared_state["robot_connected"] = False
        await server.send_connection_result(ws, False, str(exc))
        print(f"Robot connection failed: {exc}")


async def on_robot_disconnect(ws) -> None:
    loop = asyncio.get_running_loop()
    path_executor.cancel()
    if robot.connected:
        await loop.run_in_executor(None, robot.end_freedrive)
    await loop.run_in_executor(None, robot.disconnect)
    with state_lock:
        shared_state["robot_connected"] = False
        shared_state["freedrive"]       = False
        shared_state["pending_workspace"] = None
        shared_state["executing"]       = False
        shared_state["phase"]           = "idle"
    if ws is not None:
        await server.send_connection_result(ws, False, "Disconnected")
    print("Robot disconnected")


async def on_last_client_disconnect() -> None:
    print("Last client disconnected — stopping camera and robot.")
    loop = asyncio.get_running_loop()
    camera_thread.stop()
    path_executor.cancel()
    if robot.connected:
        await loop.run_in_executor(None, robot.end_freedrive)
        await loop.run_in_executor(None, robot.disconnect)
        with state_lock:
            shared_state["robot_connected"] = False
            shared_state["freedrive"]       = False
    os.kill(os.getpid(), signal.SIGINT)


# ── Workspace setup callbacks ─────────────────────────────────────────────────
# The interactive 3-point (P0/Px/Py) freedrive calibration was removed: the
# drawing target is now always a surface mesh loaded from file (or the Test
# Mode synthetic workspace). WorkspaceConfig remains for the planar fallback.
async def on_simulate_workspace() -> None:
    """
    Set a synthetic workspace so the depth→groove→Path-Preview pipeline can be
    tested without a robot. Does not connect or move the robot; Run remains
    gated on a real connection.
    """
    ws_cfg = WorkspaceConfig.simulation()
    with state_lock:
        shared_state["workspace"]         = ws_cfg
        shared_state["pending_workspace"] = None
        shared_state["ws_points"]         = {"p0": None, "px": None, "py": None}
        shared_state["phase"]             = "previewing"
    if not camera_thread.running:
        camera_thread.start()
    print(
        f"Simulation workspace active (no robot): "
        f"{ws_cfg.x_extent:.3f} m × {ws_cfg.y_extent:.3f} m — Capture enabled."
    )


# ── Capture image / Edit / Generate path callbacks ───────────────────────────
def _reach_flags(strokes: list[list[list[float]]]) -> tuple[list[list[int]], int, int]:
    """
    Estimate which waypoints the arm can reach: inside a UR_REACH_M sphere
    around the base and outside a thin UR_MIN_REACH_M cylinder around the base
    axis. Returns (per-stroke 0/1 flags where 1 = unreachable, n_out, n_total).
    Envelope check only — joint limits/wrist configuration are not modelled.
    """
    flags: list[list[int]] = []
    n_out = n_total = 0
    for stroke in strokes:
        f = []
        for p in stroke:
            r = math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
            r_xy = math.hypot(p[0], p[1])
            bad = int(r > UR_REACH_M or r_xy < UR_MIN_REACH_M)
            f.append(bad)
            n_out += bad
            n_total += 1
        flags.append(f)
    return flags, n_out, n_total


def _mm_per_px(workspace, surface_model=None) -> float | None:
    """
    Millimetres per depth pixel for the mm-based groove filters. Planar mode
    derives it from the workspace calibration; surface mode from the drawing's
    fit onto the mesh — so neither strictly requires the other.
    """
    if workspace is not None:
        return (workspace.x_extent / DEPTH_WIDTH) * 1000.0
    if surface_model is not None:
        return surface_model.drawing_mm_per_px(DEPTH_WIDTH, DEPTH_HEIGHT)
    return None


async def on_set_groove_params(params: dict) -> None:
    """Push the latest Detect-Grooves params + crop to the camera thread so the
    LIVE depth/groove feeds update in real time (used before an image is captured).
    The crop restricts the live groove/mask preview to the selected region."""
    gp = DepthGrooveParams.from_dict(params.get("adjustments"))
    crop = Crop.from_dict(params.get("crop"))
    with state_lock:
        workspace = shared_state.get("workspace")
        surface_model = shared_state.get("surface_model")
    camera_thread.set_live_params(gp)
    camera_thread.set_live_crop(crop)
    camera_thread.set_scale(_mm_per_px(workspace, surface_model))


async def on_set_reference(ws) -> None:
    """Capture the current (undrawn) sand as a baseline for background subtraction."""
    captured = camera_thread.capture_frame()
    if captured is None:
        await server.send_reference_status(ws, False, "No depth frame to set as reference.")
        return
    depth_m, _valid, _rgb = captured
    with state_lock:
        shared_state["reference_depth"] = depth_m
    camera_thread.set_reference(depth_m)
    await server.send_reference_status(ws, True, "Reference captured — natural grooves can be subtracted.")
    print("Reference depth captured for background subtraction.")


async def on_clear_reference(ws) -> None:
    with state_lock:
        shared_state["reference_depth"] = None
    camera_thread.set_reference(None)
    await server.send_reference_status(ws, False, "Reference cleared.")
    print("Reference depth cleared.")


# ── Target surface (3D projection) callbacks ─────────────────────────────────
async def on_surface_upload(filename: str, blob: bytes) -> dict:
    """Save an uploaded STL/OBJ, load it, and broadcast mesh + status to clients."""
    SURFACE_DIR.mkdir(exist_ok=True)
    safe_name = os.path.basename(filename)
    path = SURFACE_DIR / safe_name
    path.write_bytes(blob)

    loop = asyncio.get_running_loop()
    model = await loop.run_in_executor(None, SurfaceModel.load, path)
    info = model.info()
    mesh_payload = await loop.run_in_executor(None, model.mesh_payload)

    with state_lock:
        shared_state["surface_model"] = model
        shared_state["surface_info"] = info
        shared_state["surface_mesh_payload"] = mesh_payload
        pose = shared_state["surface_pose"]
        offset = shared_state["surface_offset_mm"]
        # A surface replaces the flat workspace for mapping, so it also unlocks
        # the capture flow — no P0/Px/Py calibration needed in surface mode.
        if shared_state["phase"] == "idle":
            shared_state["phase"] = "previewing"

    if not camera_thread.running:
        camera_thread.start()

    await server.broadcast_surface_status(
        loaded=True, info=info, pose=pose, offset_mm=offset, mesh=mesh_payload,
        message=f"Surface loaded: {info['name']} ({info['faces']} faces, "
                f"{info['bbox']['size'][0]}×{info['bbox']['size'][1]} m)",
    )
    print(f"[surface] loaded {info['name']}: {info['faces']} faces, bbox {info['bbox']['size']} m")
    return {"info": info}


async def on_set_surface_pose(params: dict) -> None:
    """Update the surface placement (base frame) and TCP normal-offset."""
    pose = SurfacePose.from_dict(params.get("pose"))
    try:
        offset = float(params.get("offset_mm", 0.0))
    except (TypeError, ValueError):
        offset = 0.0
    offset = min(max(offset, -20.0), 100.0)
    with state_lock:
        shared_state["surface_pose"] = pose.to_dict()
        shared_state["surface_offset_mm"] = offset


async def on_clear_surface() -> None:
    with state_lock:
        shared_state["surface_model"] = None
        shared_state["surface_info"] = None
        shared_state["surface_mesh_payload"] = None
        shared_state["strokes_surface"] = False
    await server.broadcast_surface_status(loaded=False, message="Surface cleared.")
    print("[surface] cleared — paths map to the flat workspace again")


async def on_capture_image(ws) -> None:
    """Freeze a temporally averaged depth (+ colour) frame and enter editing."""
    # If a projector is running, blank it and wait for the rolling depth buffer
    # to refill with projector-free frames — the average uses the PAST second,
    # so blanking without the wait would not help.
    with state_lock:
        proj = shared_state.get("projection_clients", 0) > 0
    if proj:
        await server.broadcast_projection_blank(True)
        await asyncio.sleep(DEPTH_AVERAGE_FRAMES / DEPTH_FPS + 0.3)

    try:
        captured = camera_thread.capture_frame()
        if captured is None:
            await server.send_capture_result(ws, False, error="No depth frame available.")
            return

        depth_m, valid, rgb = captured
        h, w = depth_m.shape[:2]
        with state_lock:
            shared_state["captured_still"] = (depth_m, valid, rgb)
            shared_state["still_dims"]     = (w, h)
            shared_state["phase"]          = "editing"
            shared_state["strokes"]        = []

        loop = asyncio.get_running_loop()
        color = await loop.run_in_executor(None, colorize_depth, depth_m, valid)
        depth_jpg = await loop.run_in_executor(None, encode_jpeg, color)
        rgb_jpg = await loop.run_in_executor(None, encode_jpeg, rgb) if rgb is not None else None
        await server.send_still(ws, depth_jpg=depth_jpg, rgb_jpg=rgb_jpg, width=w, height=h)
        print(f"Captured still: {w}×{h} (depth+colour) — ready for crop/adjust")
    finally:
        if proj:
            await server.broadcast_projection_blank(False)


async def on_preview_adjust(ws, params: dict) -> None:
    """Reprocess the captured depth with the latest crop/groove params → preview."""
    with state_lock:
        still = shared_state.get("captured_still")
    if still is None:
        return

    with state_lock:
        reference = shared_state.get("reference_depth")
        workspace = shared_state.get("workspace")
        surface_model = shared_state.get("surface_model")

    depth_m, valid, _rgb = still
    crop   = Crop.from_dict(params.get("crop"))
    gp     = DepthGrooveParams.from_dict(params.get("adjustments"))
    mmpp   = _mm_per_px(workspace, surface_model)

    loop = asyncio.get_running_loop()
    try:
        processed = await loop.run_in_executor(
            None, process_depth, depth_m, valid, crop, gp, reference, mmpp
        )
    except Exception as exc:
        print(f"[preview] processing error: {exc}")
        return

    depth_jpg   = await loop.run_in_executor(None, encode_jpeg, processed.color_full)
    grooves_jpg = await loop.run_in_executor(None, encode_jpeg, processed.grooves)
    mask_jpg    = await loop.run_in_executor(None, encode_jpeg, processed.mask)

    # Crop the captured RGB to the same region so the RGB view shows only the crop.
    rgb_jpg = None
    if _rgb is not None:
        x0, y0 = processed.origin
        gh, gw = processed.grooves.shape[:2]
        rgb_crop = _rgb[y0:y0 + gh, x0:x0 + gw]
        rgb_jpg = await loop.run_in_executor(None, encode_jpeg, rgb_crop)

    await server.send_preview(
        ws, depth_jpg=depth_jpg, grooves_jpg=grooves_jpg, mask_jpg=mask_jpg, rgb_jpg=rgb_jpg
    )


async def on_generate_path(ws, params: dict) -> None:
    """Run groove extraction on the cropped depth and build the 3D path preview."""
    with state_lock:
        workspace     = shared_state.get("workspace")
        still         = shared_state.get("captured_still")
        dims          = shared_state.get("still_dims")
        reference     = shared_state.get("reference_depth")
        surface_model = shared_state.get("surface_model")
        surface_pose  = SurfacePose.from_dict(shared_state.get("surface_pose"))
        surface_offset = shared_state.get("surface_offset_mm", 0.0)

    if workspace is None and surface_model is None:
        await server.send_capture_result(ws, False, error="No workspace configured.")
        return
    if still is None:
        await server.send_capture_result(ws, False, error="No captured depth — press Capture first.")
        return

    depth_m, valid, _rgb = still
    width, height  = dims
    crop = Crop.from_dict(params.get("crop"))
    gp   = DepthGrooveParams.from_dict(params.get("adjustments"))
    mmpp = _mm_per_px(workspace, surface_model)

    loop = asyncio.get_running_loop()
    try:
        processed = await loop.run_in_executor(
            None, process_depth, depth_m, valid, crop, gp, reference, mmpp
        )
        extracted = await loop.run_in_executor(
            None, extract_from_edges, processed.grooves, CONTOUR_MIN_PIXELS, processed.origin
        )
    except Exception as exc:
        await server.send_capture_result(ws, False, error=str(exc))
        return

    if surface_model is not None:
        # Project the 2D drawing onto the 3D surface: waypoints lie on the mesh,
        # TCP perpendicular to it, offset applied along the surface normal.
        try:
            robot_strokes = await loop.run_in_executor(
                None, surface_model.project_strokes,
                extracted.strokes, width, height, surface_pose, surface_offset / 1000.0,
            )
        except Exception as exc:
            await server.send_capture_result(ws, False, error=f"Surface projection: {exc}")
            return
        surface_mode = True
    else:
        robot_strokes = pixels_to_robot_coords(
            extracted.strokes, workspace, width, height, draw_z_offset=0.0
        )
        surface_mode = False

    # Convert to serialisable list-of-lists for the browser's 3D preview
    strokes_data = [
        [[round(v, 5) for v in pose] for pose in stroke]
        for stroke in robot_strokes
    ]

    reach_flags, reach_out, reach_total = _reach_flags(robot_strokes)

    with state_lock:
        shared_state["strokes"] = robot_strokes
        shared_state["strokes_surface"] = surface_mode
        shared_state["phase"]   = "captured" if robot_strokes else "editing"

    await server.send_capture_result(
        ws,
        success=True,
        stroke_count=extracted.total_strokes,
        point_count=extracted.total_points,
        strokes_data=strokes_data,
        reach_flags=reach_flags,
        reach_out=reach_out,
    )
    print(f"Generated path: {extracted.total_strokes} strokes, {extracted.total_points} points"
          + (f" — WARNING: {reach_out}/{reach_total} waypoints outside estimated reach"
             if reach_out else ""))


async def on_retake(ws) -> None:
    """Discard the captured still and return to the live preview phase."""
    with state_lock:
        shared_state["captured_still"] = None
        shared_state["still_dims"]     = None
        shared_state["strokes"]        = []
        shared_state["phase"]          = "previewing"
    if not camera_thread.running:
        camera_thread.start()
    print("Retake — back to live preview")


async def on_run(ws, params: dict | None = None) -> None:
    with state_lock:
        strokes      = shared_state.get("strokes", [])
        connected    = shared_state.get("robot_connected", False)
        surface_mode = shared_state.get("strokes_surface", False)

    if not strokes:
        return
    if not connected:
        await server.send_capture_result(ws, False, error="Robot not connected.")
        return
    if path_executor.running:
        return

    # Execution settings from the preview bar: speed as % of the robot's max
    # TCP speed, plus run-time offset / safety retract distances in mm.
    params = params or {}

    def _num(key, default, lo, hi):
        try:
            return min(max(float(params.get(key, default)), lo), hi)
        except (TypeError, ValueError):
            return default

    speed_pct = _num("speed_pct", (DRAW_SPEED / MAX_TCP_SPEED) * 100.0, 1.0, 100.0)
    offset_mm = _num("offset_mm", 0.0, -20.0, 200.0)
    safety_mm = _num("safety_mm", TRAVEL_Z * 1000.0, 5.0, 300.0)
    draw_speed = (speed_pct / 100.0) * MAX_TCP_SPEED

    # Surface strokes already carry contact depth along the surface normal, so
    # the executor must not add the planar DRAW_Z on top.
    path_executor.start(
        strokes,
        draw_z=0.0 if surface_mode else DRAW_Z,
        draw_speed=draw_speed,
        normal_offset=offset_mm / 1000.0,
        travel_dist=safety_mm / 1000.0,
    )
    print(f"[executor] starting path: {len(strokes)} strokes, "
          f"{speed_pct:.0f}% speed ({draw_speed:.3f} m/s), "
          f"offset {offset_mm:.1f} mm, safety {safety_mm:.0f} mm"
          + (" (surface mode)" if surface_mode else ""))


async def on_cancel(ws) -> None:
    path_executor.cancel()
    print("[executor] cancel requested")


# ── Entry point ───────────────────────────────────────────────────────────────
server = Server(
    shared_state,
    state_lock,
    robot,
    on_connect=on_robot_connect,
    on_disconnect=on_robot_disconnect,
    on_last_disconnect=on_last_client_disconnect,
    on_simulate_workspace=on_simulate_workspace,
    on_capture_image=on_capture_image,
    on_preview_adjust=on_preview_adjust,
    on_generate_path=on_generate_path,
    on_retake=on_retake,
    on_run=on_run,
    on_cancel=on_cancel,
    on_set_groove_params=on_set_groove_params,
    on_set_reference=on_set_reference,
    on_clear_reference=on_clear_reference,
    on_surface_upload=on_surface_upload,
    on_set_surface_pose=on_set_surface_pose,
    on_clear_surface=on_clear_surface,
)

# Camera starts immediately so both MJPEG feeds are live from the moment you open the browser
camera_thread.start()


# ── Live TCP poller ───────────────────────────────────────────────────────────
# Nothing else writes shared_state["ee"], so without this the preview's TCP dot
# would sit at the origin forever. Poll the RTDE receive buffer at 10 Hz while
# connected; the broadcast loop relays it to the browser.
def _ee_poller() -> None:
    import time
    while True:
        time.sleep(0.1)
        if not robot.connected:
            continue
        try:
            pose = robot.get_ee_position()
            with state_lock:
                shared_state["ee"] = list(pose)
        except Exception:
            pass


threading.Thread(target=_ee_poller, daemon=True, name="ee_poller").start()


async def _open_browser() -> None:
    await asyncio.sleep(1.0)
    webbrowser.open(f"http://{HTTP_HOST}:{HTTP_PORT}")


async def _main() -> None:
    asyncio.create_task(_open_browser())
    await server.start()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        print("\nShutting down.")
        camera_thread.stop()
        path_executor.cancel()
        robot.end_freedrive()
        robot.disconnect()
