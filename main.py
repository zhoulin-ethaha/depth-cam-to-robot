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
async def on_set_groove_params(params: dict) -> None:
    """Push the latest Detect-Grooves params + crop to the camera thread so the
    LIVE depth/groove feeds update in real time (used before an image is captured).
    The crop restricts the live groove/mask preview to the selected region."""
    gp = DepthGrooveParams.from_dict(params.get("adjustments"))
    crop = Crop.from_dict(params.get("crop"))
    camera_thread.set_live_params(gp)
    camera_thread.set_live_crop(crop)


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

    depth_m, valid, _rgb = still
    crop   = Crop.from_dict(params.get("crop"))
    gp     = DepthGrooveParams.from_dict(params.get("adjustments"))

    loop = asyncio.get_running_loop()
    try:
        processed = await loop.run_in_executor(None, process_depth, depth_m, valid, crop, gp)
    except Exception as exc:
        print(f"[preview] processing error: {exc}")
        return

    depth_jpg   = await loop.run_in_executor(None, encode_jpeg, processed.color_full)
    grooves_jpg = await loop.run_in_executor(None, encode_jpeg, processed.grooves)
    mask_jpg    = await loop.run_in_executor(None, encode_jpeg, processed.mask)
    await server.send_preview(ws, depth_jpg=depth_jpg, grooves_jpg=grooves_jpg, mask_jpg=mask_jpg)


async def on_generate_path(ws, params: dict) -> None:
    """Run groove extraction on the cropped depth and build the 3D path preview."""
    with state_lock:
        workspace = shared_state.get("workspace")
        still     = shared_state.get("captured_still")
        dims      = shared_state.get("still_dims")

    if workspace is None:
        await server.send_capture_result(ws, False, error="No workspace configured.")
        return
    if still is None:
        await server.send_capture_result(ws, False, error="No captured depth — press Capture first.")
        return

    depth_m, valid, _rgb = still
    width, height  = dims
    crop = Crop.from_dict(params.get("crop"))
    gp   = DepthGrooveParams.from_dict(params.get("adjustments"))

    loop = asyncio.get_running_loop()
    try:
        processed = await loop.run_in_executor(None, process_depth, depth_m, valid, crop, gp)
        extracted = await loop.run_in_executor(
            None, extract_from_edges, processed.grooves, CONTOUR_MIN_PIXELS, processed.origin
        )
    except Exception as exc:
        await server.send_capture_result(ws, False, error=str(exc))
        return

    robot_strokes = pixels_to_robot_coords(
        extracted.strokes, workspace, width, height, draw_z_offset=0.0
    )

    # Convert to serialisable list-of-lists for the browser's 3D preview
    strokes_data = [
        [[round(v, 5) for v in pose] for pose in stroke]
        for stroke in robot_strokes
    ]

    with state_lock:
        shared_state["strokes"] = robot_strokes
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
        strokes   = shared_state.get("strokes", [])
        connected = shared_state.get("robot_connected", False)

    if not strokes:
        return
    if not connected:
        await server.send_capture_result(ws, False, error="Robot not connected.")
        return
    if path_executor.running:
        return

    path_executor.start(strokes)
    print(f"[executor] starting path: {len(strokes)} strokes")


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
