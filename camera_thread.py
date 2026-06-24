import threading
from collections import deque
from typing import Optional

import cv2
import numpy as np

from config import (
    DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS, DEPTH_AVERAGE_FRAMES,
)
from depth_extractor import (
    DepthGrooveParams, colorize_depth, grooves_from_depth, encode_jpeg,
)

# How often (in frames) to recompute the live groove preview. Groove detection
# is heavier than colorizing, and the sand is static, so we don't need it every
# frame — the captured still gets a clean, fully-thinned pass anyway.
_LIVE_GROOVE_EVERY = 4


class DepthCameraThread:
    """
    Captures depth frames from an Intel RealSense (D435i) and produces two JPEG
    streams:
      - depth:   the depth map colorized so depth reads as colour (the live view)
      - grooves: detected groove centrelines (live preview of what gets drawn)

    Both streams are stored in shared_state and served as MJPEG by the server.
    The raw metric depth of the most recent frames is buffered so Capture can
    return a temporally averaged frame — the single biggest win for resolving
    sub-millimetre grooves. A separate lock guards the buffer so capture_frame()
    doesn't block MJPEG delivery.
    """

    def __init__(self, shared_state: dict, state_lock: threading.Lock) -> None:
        self._state = shared_state
        self._state_lock = state_lock
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        # Rolling buffer of (depth_m float32, valid bool) for temporal averaging.
        self._buffer: deque[tuple[np.ndarray, np.ndarray]] = deque(maxlen=DEPTH_AVERAGE_FRAMES)
        self._live_params = DepthGrooveParams()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, index: Optional[int] = None) -> None:
        # `index` is accepted for API compatibility but ignored — the RealSense is
        # selected by the SDK, not an OpenCV device index.
        if self.running:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="depth_camera_thread")
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def capture_frame(self) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """
        Return a temporally averaged (depth_m, valid) over the buffered frames, or
        None if no frame has arrived yet. Used by Capture; averaging cuts per-pixel
        depth noise by ~sqrt(n_frames).
        """
        with self._frame_lock:
            if not self._buffer:
                return None
            frames = list(self._buffer)

        acc = np.zeros_like(frames[0][0], dtype=np.float32)
        cnt = np.zeros_like(acc, dtype=np.float32)
        for z, ok in frames:
            acc[ok] += z[ok]
            cnt += ok
        valid = cnt > 0
        depth_m = np.zeros_like(acc)
        depth_m[valid] = acc[valid] / cnt[valid]
        return depth_m, valid

    def _run(self) -> None:
        try:
            import pyrealsense2 as rs  # noqa: local import so the module loads without the SDK
        except ImportError:
            print("[depth] ERROR: pyrealsense2 not installed — run `pip install pyrealsense2`")
            return

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, DEPTH_FPS)

        try:
            profile = pipe.start(cfg)
        except Exception as exc:
            print(f"[depth] ERROR: cannot start RealSense depth stream: {exc}")
            return

        scale = profile.get_device().first_depth_sensor().get_depth_scale()  # metres/unit
        print(f"[depth] started RealSense {DEPTH_WIDTH}×{DEPTH_HEIGHT}@{DEPTH_FPS} "
              f"(depth scale {scale:.6f} m/unit)")

        frame_i = 0
        last_groove_jpg: Optional[bytes] = None
        try:
            while not self._stop_event.is_set():
                try:
                    frames = pipe.wait_for_frames(2000)
                except Exception:
                    continue
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue

                z = np.asarray(depth_frame.get_data(), dtype=np.float32) * scale  # metres
                ok = z > 0

                # Colorized depth — the live "depth" view.
                color = colorize_depth(z, ok, self._live_params.near_m, self._live_params.far_m)
                ok_color, color_jpg = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80])

                # Live groove preview (throttled; skeleton=False keeps it cheap).
                if frame_i % _LIVE_GROOVE_EVERY == 0:
                    groove = grooves_from_depth(z, ok, self._live_params, skeleton=False)
                    gj = encode_jpeg(groove)
                    if gj is not None:
                        last_groove_jpg = gj
                frame_i += 1

                if ok_color:
                    with self._state_lock:
                        self._state["last_depth_color_jpg"] = color_jpg.tobytes()
                        self._state["last_groove_jpg"] = last_groove_jpg

                # Buffer raw metric depth for averaged Capture.
                with self._frame_lock:
                    self._buffer.append((z, ok))
        finally:
            pipe.stop()
            with self._state_lock:
                self._state["last_depth_color_jpg"] = None
                self._state["last_groove_jpg"] = None
            with self._frame_lock:
                self._buffer.clear()
            print("[depth] stopped")
