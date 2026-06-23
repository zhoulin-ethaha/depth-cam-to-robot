import asyncio
import base64
import json
import threading
from pathlib import Path
from typing import Callable, Optional

from aiohttp import web

from config import (
    HTTP_HOST, HTTP_PORT, VIS_INTERVAL,
    CAMERA_RAW_PATH, CAMERA_PROCESSED_PATH, WS_PATH, STATIC_PATH,
)
from settings import load_settings

_VIEWER_DIR = Path(__file__).parent / "viewer"


@web.middleware
async def _no_cache_static(request: web.Request, handler):
    """Serve viewer assets with no-cache so code edits show up on a plain refresh."""
    resp = await handler(request)
    if request.path.startswith(STATIC_PATH) and not resp.prepared:
        resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return resp


class Server:
    def __init__(
        self,
        shared_state: dict,
        state_lock: threading.Lock,
        robot,
        on_connect: Callable,
        on_disconnect: Callable,
        on_last_disconnect: Optional[Callable] = None,
        on_start_freedrive: Optional[Callable] = None,
        on_end_freedrive: Optional[Callable] = None,
        on_record_point: Optional[Callable] = None,
        on_confirm_workspace: Optional[Callable] = None,
        on_reset_workspace: Optional[Callable] = None,
        on_simulate_workspace: Optional[Callable] = None,
        on_use_workspace: Optional[Callable] = None,
        on_capture_image: Optional[Callable] = None,
        on_preview_adjust: Optional[Callable] = None,
        on_generate_path: Optional[Callable] = None,
        on_retake: Optional[Callable] = None,
        on_run: Optional[Callable] = None,
        on_cancel: Optional[Callable] = None,
        on_select_camera: Optional[Callable] = None,
    ):
        self._state = shared_state
        self._lock = state_lock
        self._robot = robot
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._on_last_disconnect = on_last_disconnect
        self._on_start_freedrive = on_start_freedrive
        self._on_end_freedrive = on_end_freedrive
        self._on_record_point = on_record_point
        self._on_confirm_workspace = on_confirm_workspace
        self._on_reset_workspace = on_reset_workspace
        self._on_simulate_workspace = on_simulate_workspace
        self._on_use_workspace = on_use_workspace
        self._on_capture_image = on_capture_image
        self._on_preview_adjust = on_preview_adjust
        self._on_generate_path = on_generate_path
        self._on_retake = on_retake
        self._on_run = on_run
        self._on_cancel = on_cancel
        self._on_select_camera = on_select_camera
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._app = self._build_app()

    def _build_app(self) -> web.Application:
        app = web.Application(middlewares=[_no_cache_static])
        app.router.add_get("/", self._handle_index)
        app.router.add_get("/cameras", self._handle_cameras)
        app.router.add_get(CAMERA_RAW_PATH, self._handle_camera_raw)
        app.router.add_get(CAMERA_PROCESSED_PATH, self._handle_camera_processed)
        app.router.add_get(WS_PATH, self._handle_ws)
        app.router.add_static(STATIC_PATH, _VIEWER_DIR, show_index=False)
        return app

    async def start(self) -> None:
        runner = web.AppRunner(self._app)
        await runner.setup()
        site = web.TCPSite(runner, HTTP_HOST, HTTP_PORT)
        await site.start()
        print(f"GUI ready → http://{HTTP_HOST}:{HTTP_PORT}")
        await self._broadcast_loop()

    async def _handle_index(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(_VIEWER_DIR / "index.html")

    async def _handle_cameras(self, request: web.Request) -> web.Response:
        with self._lock:
            cameras = self._state.get("available_cameras", [])
        return web.json_response(cameras)

    async def _mjpeg_stream(self, request: web.Request, key: str) -> web.StreamResponse:
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)
        try:
            while True:
                with self._lock:
                    jpg = self._state.get(key)
                if jpg:
                    payload = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" +
                        jpg +
                        b"\r\n"
                    )
                    await response.write(payload)
                await asyncio.sleep(1 / 30)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def _handle_camera_raw(self, request: web.Request) -> web.StreamResponse:
        return await self._mjpeg_stream(request, "last_frame_raw_jpg")

    async def _handle_camera_processed(self, request: web.Request) -> web.StreamResponse:
        return await self._mjpeg_stream(request, "last_frame_canny_jpg")

    async def _handle_ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.add(ws)

        settings = load_settings()
        with self._lock:
            ws_cfg = self._state.get("workspace")
        await self._send_init(ws, settings.get("last_ip", ""), ws_cfg)

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    await self._handle_ws_message(ws, msg.data)
        finally:
            self._ws_clients.discard(ws)
            if not self._ws_clients and self._on_last_disconnect:
                asyncio.create_task(self._on_last_disconnect())

        return ws

    async def _send_init(self, ws, last_ip: str, ws_cfg) -> None:
        try:
            await ws.send_str(json.dumps({
                "type": "init",
                "last_ip": last_ip,
                "workspace": ws_cfg.to_browser_dict() if ws_cfg is not None else None,
            }))
        except Exception:
            pass

    async def _handle_ws_message(self, ws, raw: str) -> None:
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        msg_type = data.get("type", "")

        if msg_type == "connect":
            ip = data.get("ip", "").strip()
            if ip:
                await self._on_connect(ip, ws)

        elif msg_type == "disconnect":
            await self._on_disconnect(ws)

        elif msg_type == "start_freedrive":
            if self._on_start_freedrive:
                asyncio.create_task(self._on_start_freedrive())

        elif msg_type == "end_freedrive":
            if self._on_end_freedrive:
                asyncio.create_task(self._on_end_freedrive())

        elif msg_type == "record_point":
            name = data.get("name", "")
            if self._on_record_point and name:
                asyncio.create_task(self._on_record_point(name))

        elif msg_type == "confirm_workspace":
            if self._on_confirm_workspace:
                asyncio.create_task(self._on_confirm_workspace())

        elif msg_type == "reset_workspace":
            if self._on_reset_workspace:
                asyncio.create_task(self._on_reset_workspace())

        elif msg_type == "use_workspace":
            if self._on_use_workspace:
                asyncio.create_task(self._on_use_workspace())

        elif msg_type == "simulate_workspace":
            if self._on_simulate_workspace:
                asyncio.create_task(self._on_simulate_workspace())

        elif msg_type == "capture_image":
            if self._on_capture_image:
                asyncio.create_task(self._on_capture_image(ws))

        elif msg_type == "preview_adjust":
            if self._on_preview_adjust:
                asyncio.create_task(self._on_preview_adjust(ws, data.get("params", {})))

        elif msg_type == "generate_path":
            if self._on_generate_path:
                asyncio.create_task(self._on_generate_path(ws, data.get("params", {})))

        elif msg_type == "retake":
            if self._on_retake:
                asyncio.create_task(self._on_retake(ws))

        elif msg_type == "run":
            if self._on_run:
                asyncio.create_task(self._on_run(ws))

        elif msg_type == "cancel":
            if self._on_cancel:
                asyncio.create_task(self._on_cancel(ws))

        elif msg_type == "select_camera":
            index = data.get("index")
            if self._on_select_camera and index is not None:
                asyncio.create_task(self._on_select_camera(int(index)))

    async def _broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(VIS_INTERVAL)
            if not self._ws_clients:
                continue

            with self._lock:
                connected  = self._state.get("robot_connected", False)
                ee         = self._state.get("ee", [0.0] * 6)
                phase      = self._state.get("phase", "idle")
                strokes    = self._state.get("strokes", [])
                executing  = self._state.get("executing", False)
                progress   = self._state.get("progress", 0.0)
                freedrive  = self._state.get("freedrive", False)
                ws_pts     = self._state.get("ws_points", {})
                ws_cfg     = self._state.get("workspace")

            msg = json.dumps({
                "type": "state",
                "robot_connected": connected,
                "ee": list(ee[:3]),
                "phase": phase,
                "stroke_count": len(strokes),
                "executing": executing,
                "progress": round(progress, 3),
                "freedrive": freedrive,
                "ws_points": {
                    k: ([round(v, 4) for v in vals] if vals is not None else None)
                    for k, vals in ws_pts.items()
                },
                "workspace": ws_cfg.to_browser_dict() if ws_cfg is not None else None,
            })

            dead = set()
            for client in list(self._ws_clients):
                try:
                    await client.send_str(msg)
                except Exception:
                    dead.add(client)
            self._ws_clients -= dead

    async def send_connection_result(self, ws, success: bool, message: str) -> None:
        try:
            await ws.send_str(json.dumps({
                "type": "connection_result",
                "success": success,
                "message": message,
            }))
        except Exception:
            pass

    async def send_workspace_status(self, ws, loaded: bool, workspace=None) -> None:
        try:
            await ws.send_str(json.dumps({
                "type": "workspace_status",
                "loaded": loaded,
                "workspace": workspace.to_browser_dict() if workspace is not None else None,
            }))
        except Exception:
            pass

    @staticmethod
    def _data_url(jpg: Optional[bytes]) -> Optional[str]:
        if not jpg:
            return None
        return "data:image/jpeg;base64," + base64.b64encode(jpg).decode("ascii")

    async def send_still(self, ws, jpg: Optional[bytes], width: int, height: int) -> None:
        """Send the frozen still image (as a data URL) plus its pixel dimensions."""
        try:
            await ws.send_str(json.dumps({
                "type": "still",
                "image": self._data_url(jpg),
                "width": width,
                "height": height,
            }))
        except Exception:
            pass

    async def send_preview(self, ws, adjusted_jpg: Optional[bytes],
                           edges_jpg: Optional[bytes]) -> None:
        """Send the live edit preview: adjusted grayscale + Canny edges (data URLs)."""
        try:
            await ws.send_str(json.dumps({
                "type": "preview",
                "adjusted": self._data_url(adjusted_jpg),
                "edges": self._data_url(edges_jpg),
            }))
        except Exception:
            pass

    async def send_capture_result(
        self,
        ws,
        success: bool,
        stroke_count: int = 0,
        point_count: int = 0,
        strokes_data: Optional[list] = None,
        error: Optional[str] = None,
    ) -> None:
        try:
            await ws.send_str(json.dumps({
                "type": "capture_result",
                "success": success,
                "stroke_count": stroke_count,
                "point_count": point_count,
                "strokes": strokes_data or [],
                "error": error,
            }))
        except Exception:
            pass

    async def broadcast_execution_update(self, phase: str, progress: float, error: Optional[str] = None) -> None:
        msg = json.dumps({
            "type": "execution_update",
            "phase": phase,
            "progress": round(progress, 3),
            "error": error,
        })
        dead = set()
        for client in list(self._ws_clients):
            try:
                await client.send_str(msg)
            except Exception:
                dead.add(client)
        self._ws_clients -= dead
