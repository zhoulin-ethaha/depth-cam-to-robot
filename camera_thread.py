import threading
from collections import deque
from typing import Optional

import cv2
import numpy as np

from config import (
    DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS, DEPTH_AVERAGE_FRAMES,
)
from depth_extractor import (
    Crop, DepthGrooveParams, colorize_depth, grooves_and_mask, encode_jpeg,
)

# How often (in frames) to recompute the live groove preview. Groove detection
# is heavier than colorizing, and the sand is static, so we don't need it every
# frame — the captured still gets a clean, fully-thinned pass anyway.
_LIVE_GROOVE_EVERY = 4


class DepthCameraThread:
    """
    Captures depth + colour frames from an Intel RealSense (D435i) and produces
    three JPEG streams:
      - depth:   the depth map colorized so depth reads as colour
      - rgb:     the aligned colour image
      - grooves: detected groove centrelines (live preview of what gets drawn)

    All three are stored in shared_state and served as MJPEG by the server. The
    colour stream is aligned to the depth frame so a crop in normalized
    coordinates selects the same region in both. The raw metric depth of recent
    frames is buffered so Capture can return a temporally averaged frame — the
    single biggest win for resolving sub-millimetre grooves. A separate lock
    guards the buffers so capture_frame() doesn't block MJPEG delivery.

    The live groove preview honours `set_live_params()` so the browser's Detect
    Grooves controls update the feed in real time, before any image is captured.
    """

    def __init__(self, shared_state: dict, state_lock: threading.Lock) -> None:
        self._state = shared_state
        self._state_lock = state_lock
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        # Rolling buffer of (depth_m float32, valid bool) for temporal averaging.
        self._buffer: deque[tuple[np.ndarray, np.ndarray]] = deque(maxlen=DEPTH_AVERAGE_FRAMES)
        self._last_rgb: Optional[np.ndarray] = None
        # Live detection params + crop (atomically swapped by the setters).
        self._live_params = DepthGrooveParams()
        self._live_crop = Crop()
        self._reference: Optional[np.ndarray] = None   # baseline depth for subtraction
        self._mm_per_px: Optional[float] = None        # workspace scale for mm filters

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

    def set_live_params(self, params: DepthGrooveParams) -> None:
        """Update the params used for the live depth colormap + groove preview."""
        self._live_params = params

    def set_live_crop(self, crop: Crop) -> None:
        """Restrict the live groove/mask preview to this normalized crop region."""
        self._live_crop = crop

    def set_reference(self, depth_m: Optional[np.ndarray]) -> None:
        """Set (or clear with None) the baseline depth frame for background subtraction."""
        self._reference = depth_m

    def set_scale(self, mm_per_px: Optional[float]) -> None:
        """Set the workspace scale so the live mm-based width/length filters work."""
        self._mm_per_px = mm_per_px

    def capture_frame(self) -> Optional[tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]]:
        """
        Return a temporally averaged (depth_m, valid, rgb) over the buffered
        frames, or None if no frame has arrived yet. ``rgb`` is the most recent
        aligned colour frame (BGR), or None if the colour stream produced nothing.
        Averaging cuts per-pixel depth noise by ~sqrt(n_frames).
        """
        with self._frame_lock:
            if not self._buffer:
                return None
            frames = list(self._buffer)
            rgb = None if self._last_rgb is None else self._last_rgb.copy()

        acc = np.zeros_like(frames[0][0], dtype=np.float32)
        cnt = np.zeros_like(acc, dtype=np.float32)
        for z, ok in frames:
            acc[ok] += z[ok]
            cnt += ok
        valid = cnt > 0
        depth_m = np.zeros_like(acc)
        depth_m[valid] = acc[valid] / cnt[valid]
        return depth_m, valid, rgb

    def _run(self) -> None:
        try:
            import pyrealsense2 as rs  # noqa: local import so the module loads without the SDK
        except ImportError:
            print("[depth] ERROR: pyrealsense2 not installed — run `pip install pyrealsense2`")
            return

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, DEPTH_FPS)
        cfg.enable_stream(rs.stream.color, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.bgr8, DEPTH_FPS)

        try:
            profile = pipe.start(cfg)
        except Exception as exc:
            print(f"[depth] ERROR: cannot start RealSense streams: {exc}")
            return

        scale = profile.get_device().first_depth_sensor().get_depth_scale()  # metres/unit
        align = rs.align(rs.stream.depth)  # bring colour into the depth pixel grid
        print(f"[depth] started RealSense {DEPTH_WIDTH}×{DEPTH_HEIGHT}@{DEPTH_FPS} "
              f"depth+colour (depth scale {scale:.6f} m/unit)")

        frame_i = 0
        last_groove_jpg: Optional[bytes] = None
        last_mask_jpg: Optional[bytes] = None
        try:
            while not self._stop_event.is_set():
                try:
                    frames = align.process(pipe.wait_for_frames(2000))
                except Exception:
                    continue
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue
                color_frame = frames.get_color_frame()

                z = np.asarray(depth_frame.get_data(), dtype=np.float32) * scale  # metres
                ok = z > 0
                params = self._live_params

                # Colorized depth (FULL frame — the crop box overlays it client-side).
                color = colorize_depth(z, ok, params.near_m, params.far_m)
                ok_color, color_jpg = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80])

                # Aligned RGB — the live "rgb" view.
                rgb = np.asarray(color_frame.get_data()) if color_frame else None
                rgb_jpg = encode_jpeg(rgb) if rgb is not None else None

                # Live groove + mask preview, restricted to the live crop (throttled).
                if frame_i % _LIVE_GROOVE_EVERY == 0:
                    h, w = z.shape[:2]
                    x0, y0, x1, y1 = self._live_crop.pixel_box(w, h)
                    ref = self._reference
                    ref_sub = ref[y0:y1, x0:x1] if (ref is not None and ref.shape == z.shape) else None
                    mask, skel = grooves_and_mask(
                        z[y0:y1, x0:x1], ok[y0:y1, x0:x1], params, ref_sub, self._mm_per_px
                    )
                    sj, mj = encode_jpeg(skel), encode_jpeg(mask)
                    if sj is not None:
                        last_groove_jpg = sj
                    if mj is not None:
                        last_mask_jpg = mj
                frame_i += 1

                if ok_color:
                    with self._state_lock:
                        self._state["last_depth_color_jpg"] = color_jpg.tobytes()
                        self._state["last_rgb_jpg"] = rgb_jpg
                        self._state["last_groove_jpg"] = last_groove_jpg
                        self._state["last_mask_jpg"] = last_mask_jpg

                # Buffer raw metric depth (+ latest RGB) for averaged Capture.
                with self._frame_lock:
                    self._buffer.append((z, ok))
                    if rgb is not None:
                        self._last_rgb = rgb
        finally:
            pipe.stop()
            with self._state_lock:
                self._state["last_depth_color_jpg"] = None
                self._state["last_rgb_jpg"] = None
                self._state["last_groove_jpg"] = None
                self._state["last_mask_jpg"] = None
            with self._frame_lock:
                self._buffer.clear()
                self._last_rgb = None
            print("[depth] stopped")
