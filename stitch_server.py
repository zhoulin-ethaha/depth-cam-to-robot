"""
aiohttp server for the dual-camera stitching prototype (port 5006).

Serves the stitch UI (viewer/stitch.html), four MJPEG streams (stitched depth /
RGB / mask / skeleton) and a small JSON WebSocket for the calibration and
detection-parameter controls. Completely separate from server.py — this tool
must stay contained from Developer/Participant Mode. Like the main app, the
process exits (SIGINT) when the last browser tab closes.
"""

from __future__ import annotations

import asyncio
import json
import os
import signal
import threading
from pathlib import Path

from aiohttp import WSMsgType, web

from config import HTTP_HOST, STITCH_CALIB_FILE, STITCH_HTTP_PORT
from depth_extractor import DepthGrooveParams
from dual_camera import DualCameraThread
from stitcher import StitchCalib

_VIEWER_DIR = Path(__file__).parent / "viewer"


class StitchServer:
    def __init__(self, camera: DualCameraThread,
                 shared_state: dict, state_lock: threading.Lock) -> None:
        self._camera = camera
        self._state = shared_state
        self._lock = state_lock
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._had_client = False
        self._params = DepthGrooveParams()
        self._app = self._build_app()

    def _build_app(self) -> web.Application:
        app = web.Application()

        @web.middleware
        async def no_cache(request, handler):
            resp = await handler(request)
            if request.path == "/":
                resp.headers["Cache-Control"] = "no-store"
            return resp

        app.middlewares.append(no_cache)
        app.router.add_get("/", self._handle_index)
        app.router.add_get("/stitch/depth", lambda r: self._mjpeg(r, "stitch_depth_jpg"))
        app.router.add_get("/stitch/rgb", lambda r: self._mjpeg(r, "stitch_rgb_jpg"))
        app.router.add_get("/stitch/mask", lambda r: self._mjpeg(r, "stitch_mask_jpg"))
        app.router.add_get("/stitch/skel", lambda r: self._mjpeg(r, "stitch_skel_jpg"))
        app.router.add_get("/ws", self._handle_ws)
        app.router.add_static("/static", _VIEWER_DIR, show_index=False)
        return app

    async def start(self) -> None:
        runner = web.AppRunner(self._app)
        await runner.setup()
        site = web.TCPSite(runner, HTTP_HOST, STITCH_HTTP_PORT)
        await site.start()
        print(f"Stitch prototype ready -> http://{HTTP_HOST}:{STITCH_HTTP_PORT}")
        await self._broadcast_loop()

    async def _handle_index(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(_VIEWER_DIR / "stitch.html")

    async def _mjpeg(self, request: web.Request, key: str) -> web.StreamResponse:
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)
        try:
            while True:
                with self._lock:
                    jpg = self._state.get(key)
                if jpg:
                    await response.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                         + jpg + b"\r\n")
                await asyncio.sleep(0.2)   # streams refresh at the stitch cadence
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    # ── WebSocket ────────────────────────────────────────────────────────────
    async def _handle_ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._ws_clients.add(ws)
        self._had_client = True
        await ws.send_str(json.dumps({
            "type": "init",
            "calib": self._camera.get_calib().to_dict(),
            "params": _params_dict(self._params),
        }))
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                await self._dispatch(ws, data.get("type"), data.get("params") or {})
        finally:
            self._ws_clients.discard(ws)
        return ws

    async def _dispatch(self, ws, mtype: str, params: dict) -> None:
        if mtype == "set_params":
            self._params = DepthGrooveParams.from_dict(params)
            self._camera.set_params(self._params)
        elif mtype == "set_calib":
            merged = {**self._camera.get_calib().to_dict(), **params}
            self._camera.set_calib(StitchCalib.from_dict(merged))
        elif mtype == "auto_refine":
            self._camera.request_refine()
        elif mtype == "save_calib":
            calib = self._camera.get_calib().to_dict()
            STITCH_CALIB_FILE.write_text(json.dumps(calib, indent=2))
            await ws.send_str(json.dumps({
                "type": "save_result", "success": True,
                "message": f"Calibration saved to {STITCH_CALIB_FILE}"}))

    # ── state broadcast + last-tab shutdown ──────────────────────────────────
    async def _broadcast_loop(self) -> None:
        empty_since = None
        last_refine = None
        while True:
            with self._lock:
                info = self._state.get("stitch_info")
                note = self._state.get("stitch_note")
                calib = self._state.get("stitch_calib")
                refine = self._state.get("stitch_refine_result")
            payload = {"type": "state", "info": info, "note": note, "calib": calib}
            if refine is not None and refine is not last_refine:
                payload["refine"] = refine
                last_refine = refine
            text = json.dumps(payload)
            for ws in list(self._ws_clients):
                try:
                    await ws.send_str(text)
                except (ConnectionResetError, RuntimeError):
                    self._ws_clients.discard(ws)

            if self._had_client and not self._ws_clients:
                empty_since = empty_since or asyncio.get_event_loop().time()
                if asyncio.get_event_loop().time() - empty_since > 2.5:
                    print("Last stitch client disconnected — shutting down.")
                    os.kill(os.getpid(), signal.SIGINT)
            else:
                empty_since = None
            await asyncio.sleep(0.25)


def _params_dict(p: DepthGrooveParams) -> dict:
    return {
        "detect": p.detect,
        "smooth_sigma_px": p.smooth_sigma_px,
        "detrend_sigma_px": p.detrend_sigma_px,
        "groove_depth_mm": p.groove_depth_mm,
        "min_blob_px": p.min_blob_px,
        "min_mean_depth_mm": p.min_mean_depth_mm,
        "min_width_mm": p.min_width_mm,
        "max_width_mm": p.max_width_mm,
        "min_length_mm": p.min_length_mm,
    }


def load_saved_calib() -> StitchCalib:
    """Read stitch_calibration.json if present, else defaults."""
    try:
        return StitchCalib.from_dict(json.loads(STITCH_CALIB_FILE.read_text()))
    except (OSError, json.JSONDecodeError, ValueError):
        return StitchCalib()
