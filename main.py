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

from config import (
    HTTP_HOST, HTTP_PORT, WORKSPACE_FILE, CONTOUR_MIN_PIXELS,
    DEPTH_WIDTH, DEPTH_HEIGHT, SURFACE_DIR,
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

        if WORKSPACE_FILE.exists():
            try:
                ws_cfg = WorkspaceConfig.load(WORKSPACE_FILE)
                with state_lock:
                    shared_state["pending_workspace"] = ws_cfg
                await server.send_workspace_status(ws, loaded=True, workspace=ws_cfg)
            except Exception as exc:
                print(f"Failed to load workspace.json: {exc}")
                await server.send_workspace_status(ws, loaded=False)
        else:
            await server.send_workspace_status(ws, loaded=False)

        await server.send_connection_result(ws, True, f"Connected to {ip}")
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
async def on_start_freedrive() -> None:
    with state_lock:
        shared_state["freedrive"] = True
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, robot.start_freedrive)
    print("Freedrive activated")


async def on_end_freedrive() -> None:
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, robot.end_freedrive)
    with state_lock:
        shared_state["freedrive"] = False
    print("Freedrive ended")


async def on_record_point(name: str) -> None:
    if name not in ("p0", "px", "py"):
        return
    pos = robot.get_ee_position()[:3]
    with state_lock:
        shared_state["ws_points"][name] = pos
    print(f"Recorded workspace point {name}: {[round(v, 4) for v in pos]}")


async def on_confirm_workspace() -> None:
    with state_lock:
        pts = shared_state["ws_points"].copy()

    p0, px, py = pts.get("p0"), pts.get("px"), pts.get("py")
    if p0 is None or px is None or py is None:
        print("Cannot confirm workspace — not all points recorded.")
        return

    try:
        ws_cfg = WorkspaceConfig.from_points(p0, px, py)
    except ValueError as exc:
        print(f"Workspace geometry error: {exc}")
        return

    ws_cfg.save(WORKSPACE_FILE)

    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, robot.end_freedrive)
    await asyncio.sleep(0.5)

    with state_lock:
        shared_state["workspace"]  = ws_cfg
        shared_state["freedrive"]  = False
        shared_state["ws_points"]  = {"p0": None, "px": None, "py": None}
        shared_state["phase"]      = "previewing"

    camera_thread.start()
    print(
        f"Workspace confirmed: {ws_cfg.x_extent:.3f} m × {ws_cfg.y_extent:.3f} m, "
        f"origin {[round(v, 3) for v in ws_cfg.origin]}"
    )


async def on_use_workspace() -> None:
    with state_lock:
        ws_cfg = shared_state.pop("pending_workspace", None)
        if ws_cfg is None:
            ws_cfg = shared_state.get("workspace")
        shared_state["workspace"] = ws_cfg
        shared_state["phase"]     = "previewing"
    camera_thread.start()
    print("Using existing workspace — camera started.")


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


async def on_reset_workspace() -> None:
    camera_thread.stop()
    if WORKSPACE_FILE.exists():
        WORKSPACE_FILE.unlink()
    with state_lock:
        shared_state["workspace"]        = None
        shared_state["pending_workspace"] = None
        shared_state["ws_points"]        = {"p0": None, "px": None, "py": None}
        shared_state["phase"]            = "idle"
        shared_state["strokes"]          = []
        shared_state["captured_still"]   = None
        shared_state["still_dims"]       = None
    print("Workspace reset — setup required again.")


# ── Capture image / Edit / Generate path callbacks ───────────────────────────
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
    )
    print(f"Generated path: {extracted.total_strokes} strokes, {extracted.total_points} points")


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


async def on_run(ws) -> None:
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

    # Surface strokes already carry contact depth along the surface normal, so
    # the executor must not add the planar DRAW_Z on top.
    if surface_mode:
        path_executor.start(strokes, draw_z=0.0)
    else:
        path_executor.start(strokes)
    print(f"[executor] starting path: {len(strokes)} strokes"
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
    on_start_freedrive=on_start_freedrive,
    on_end_freedrive=on_end_freedrive,
    on_record_point=on_record_point,
    on_confirm_workspace=on_confirm_workspace,
    on_reset_workspace=on_reset_workspace,
    on_simulate_workspace=on_simulate_workspace,
    on_use_workspace=on_use_workspace,
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
