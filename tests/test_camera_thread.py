"""
Unit tests for camera_thread.py — CameraThread with mocked cv2.VideoCapture.
No physical camera required.
"""
import threading
import time
from unittest.mock import MagicMock, patch, call

import cv2
import numpy as np
import pytest

from camera_thread import CameraThread


def _make_fake_cap(frame: np.ndarray = None, opened: bool = True):
    """Helper: configure a MagicMock VideoCapture."""
    if frame is None:
        frame = np.ones((10, 10, 3), dtype=np.uint8) * 200  # light gray
    cap = MagicMock()
    cap.isOpened.return_value = opened
    cap.read.return_value = (True, frame)
    cap.get.side_effect = lambda prop: {
        cv2.CAP_PROP_FRAME_WIDTH: 640.0,
        cv2.CAP_PROP_FRAME_HEIGHT: 480.0,
    }.get(prop, 0.0)
    return cap


# ─────────────────────────────────────────────────────────────────────────────
# Lifecycle
# ─────────────────────────────────────────────────────────────────────────────

class TestCameraThreadLifecycle:

    def test_not_running_before_start(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        ct = CameraThread(state, lock)
        assert ct.running is False

    def test_running_after_start(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.05)
            assert ct.running is True
            ct.stop()

    def test_not_running_after_stop(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.05)
            ct.stop()
            assert ct.running is False

    def test_start_idempotent(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        with patch("camera_thread.cv2.VideoCapture") as mock_vc:
            mock_vc.return_value = cap
            ct = CameraThread(state, lock)
            ct.start()
            ct.start()  # second call must be a no-op
            time.sleep(0.05)
            ct.stop()
            assert mock_vc.call_count == 1

    def test_stop_before_start_is_safe(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        ct = CameraThread(state, lock)
        ct.stop()  # must not raise


# ─────────────────────────────────────────────────────────────────────────────
# Shared state population
# ─────────────────────────────────────────────────────────────────────────────

class TestSharedStatePopulation:

    def test_shared_state_populated_with_jpeg_bytes(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.08)
            # Assert before stop() — stop() clears both JPEG keys to None
            assert isinstance(state["last_frame_raw_jpg"], bytes)
            assert isinstance(state["last_frame_canny_jpg"], bytes)
            ct.stop()

    def test_raw_jpeg_is_valid(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.08)
            jpg = state["last_frame_raw_jpg"]
            ct.stop()
        decoded = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None
        assert decoded.ndim == 3

    def test_canny_jpeg_is_valid(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.08)
            jpg = state["last_frame_canny_jpg"]
            ct.stop()
        decoded = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
        assert decoded is not None

    def test_stop_clears_shared_state(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.08)
            ct.stop()
        assert state["last_frame_raw_jpg"] is None
        assert state["last_frame_canny_jpg"] is None


# ─────────────────────────────────────────────────────────────────────────────
# Known bugs / failure modes
# ─────────────────────────────────────────────────────────────────────────────

class TestKnownBehaviors:

    def test_camera_unavailable_thread_exits_silently(self, shared_state_and_lock):
        """
        BUG DOCUMENTATION: when the camera is unavailable, the thread exits
        silently (just prints an error). No signal is raised to the application.
        The JPEG streams remain None.
        """
        state, lock = shared_state_and_lock
        cap = _make_fake_cap(opened=False)
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.15)
            # Thread exits on its own because isOpened() is False
            assert ct.running is False
        assert state["last_frame_raw_jpg"] is None
        assert state["last_frame_canny_jpg"] is None

    def test_frame_read_failure_serves_stale_jpeg(self, shared_state_and_lock):
        """
        BUG DOCUMENTATION: when cap.read() fails after a good first frame,
        the old JPEG remains in shared_state. No error is raised.
        """
        state, lock = shared_state_and_lock
        good_frame = np.ones((10, 10, 3), dtype=np.uint8) * 200
        cap = _make_fake_cap(frame=good_frame)
        # Return the good frame once, then always fail — use a function to avoid
        # StopIteration leaking out of the mock when the list is exhausted.
        call_count = [0]
        def _read():
            call_count[0] += 1
            return (True, good_frame) if call_count[0] == 1 else (False, None)
        cap.read.side_effect = _read
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.1)
            # Thread is alive but stuck returning (False, None) — check state before stop
            assert state["last_frame_raw_jpg"] is not None, "stale JPEG should be preserved"
            ct.stop()


# ─────────────────────────────────────────────────────────────────────────────
# capture_frame
# ─────────────────────────────────────────────────────────────────────────────

class TestCaptureFrame:

    def test_returns_none_before_first_frame(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        ct = CameraThread(state, lock)
        assert ct.capture_frame() is None

    def test_returns_numpy_array_after_start(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.08)
            frame = ct.capture_frame()
            ct.stop()
        assert isinstance(frame, np.ndarray)
        assert frame.ndim == 3

    def test_returns_copy_not_reference(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.08)
            f1 = ct.capture_frame()
            f2 = ct.capture_frame()
            ct.stop()
        assert f1 is not f2

    def test_concurrent_capture_frame_no_errors(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        cap = _make_fake_cap()
        errors = []

        def reader(ct):
            for _ in range(50):
                try:
                    ct.capture_frame()
                except Exception as exc:
                    errors.append(exc)

        with patch("camera_thread.cv2.VideoCapture", return_value=cap):
            ct = CameraThread(state, lock)
            ct.start()
            time.sleep(0.05)
            threads = [threading.Thread(target=reader, args=(ct,)) for _ in range(10)]
            for t in threads:
                t.start()
            for t in threads:
                t.join(timeout=2.0)
            ct.stop()

        assert errors == [], f"Concurrent access raised: {errors}"
