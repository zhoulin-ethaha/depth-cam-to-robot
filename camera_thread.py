import threading
from typing import Optional

import cv2
import numpy as np

from config import (
    CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT,
    CANNY_BLUR_KERNEL, CANNY_THRESHOLD_LOW, CANNY_THRESHOLD_HIGH,
)


class CameraThread:
    """
    Captures frames from the external webcam and produces two JPEG streams:
    - raw: colour camera feed
    - Canny: edge-detected view for drawing preview

    Both streams are stored in shared_state and served as MJPEG by the server.
    A separate lock guards _last_frame so capture_frame() doesn't block MJPEG delivery.
    """

    def __init__(self, shared_state: dict, state_lock: threading.Lock) -> None:
        self._state = shared_state
        self._state_lock = state_lock
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._last_frame: Optional[np.ndarray] = None
        self._index: int = CAMERA_INDEX

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, index: Optional[int] = None) -> None:
        if self.running:
            return
        if index is not None:
            self._index = index
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="camera_thread")
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def capture_frame(self) -> Optional[np.ndarray]:
        """Return a copy of the most recent BGR frame (thread-safe). Used by Capture."""
        with self._frame_lock:
            if self._last_frame is None:
                return None
            return self._last_frame.copy()

    def _run(self) -> None:
        cap = cv2.VideoCapture(self._index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not cap.isOpened():
            print(f"[camera] ERROR: cannot open camera index {self._index}")
            return

        print(f"[camera] started on index {self._index} "
              f"({int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}×"
              f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))})")

        while not self._stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                continue

            # Raw JPEG
            ok_raw, raw_jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

            # Canny edge map
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (CANNY_BLUR_KERNEL, CANNY_BLUR_KERNEL), 0)
            edges   = cv2.Canny(blurred, CANNY_THRESHOLD_LOW, CANNY_THRESHOLD_HIGH)
            ok_edge, canny_jpg = cv2.imencode(".jpg", edges, [cv2.IMWRITE_JPEG_QUALITY, 80])

            # Store both JPEGs for MJPEG endpoints
            if ok_raw and ok_edge:
                with self._state_lock:
                    self._state["last_frame_raw_jpg"]   = raw_jpg.tobytes()
                    self._state["last_frame_canny_jpg"] = canny_jpg.tobytes()

            # Store raw BGR for one-shot Capture
            with self._frame_lock:
                self._last_frame = frame

        cap.release()
        with self._state_lock:
            self._state["last_frame_raw_jpg"]   = None
            self._state["last_frame_canny_jpg"] = None
        print("[camera] stopped")
