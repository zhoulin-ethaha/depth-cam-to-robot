"""
Entry point for the Dual-Cam Vision prototype (dual-camera stitching).

Run with run_stitch.bat (or the sandskript conda env's python stitch_main.py) →
http://localhost:5006. CONTAINED from the main app: do not run both at once —
each RealSense can only be owned by one process. With fewer than two cameras
attached the tool runs on a synthetic scene so the UI and calibration workflow
can still be exercised.

Never `import main` here (it starts the main app's camera thread).
"""

from __future__ import annotations

import asyncio
import threading
import webbrowser

from config import HTTP_HOST, STITCH_CALIB_FILE, STITCH_HTTP_PORT
from dual_camera import DualCameraThread
from stitch_server import StitchServer, load_saved_calib

shared_state: dict = {
    "stitch_depth_jpg": None,
    "stitch_rgb_jpg": None,
    "stitch_mask_jpg": None,
    "stitch_skel_jpg": None,
    "stitch_left_depth_jpg": None,
    "stitch_left_rgb_jpg": None,
    "stitch_right_depth_jpg": None,
    "stitch_right_rgb_jpg": None,
    "stitch_info": None,
    "stitch_note": None,
    "stitch_calib": None,
    "stitch_refine_result": None,
    "stitch_on": None,
}
state_lock = threading.Lock()

camera = DualCameraThread(shared_state, state_lock)
server = StitchServer(camera, shared_state, state_lock)


async def _main() -> None:
    camera.set_calib(load_saved_calib())
    # Calibrated rigs jump straight to the stitched view; a fresh setup starts
    # on the per-camera screen so the feeds can be oriented first.
    camera.set_stitch(STITCH_CALIB_FILE.exists())
    camera.start()
    asyncio.get_running_loop().call_later(
        1.0, webbrowser.open, f"http://{HTTP_HOST}:{STITCH_HTTP_PORT}")
    await server.start()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
