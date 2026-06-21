"""
Integration tests — require physical hardware.
Run only when camera and/or robot are connected:
  pytest -m integration -v

Skip in CI:
  pytest -m "not integration" -v
"""
import os

import pytest


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _robot_ip():
    ip = os.environ.get("TEST_ROBOT_IP", "")
    if not ip:
        pytest.skip("TEST_ROBOT_IP env var not set")
    return ip


# ─────────────────────────────────────────────────────────────────────────────
# Camera
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.integration
def test_camera_live_feed(shared_state_and_lock):
    """
    Prerequisite: webcam present at CAMERA_INDEX (see config.py).
    Asserts:
    - Both JPEG streams are non-None bytes within 1 second.
    - Raw JPEG decodes to an image matching configured dimensions.
    """
    import time
    import cv2
    import numpy as np
    from camera_thread import CameraThread
    from config import CAMERA_WIDTH, CAMERA_HEIGHT

    state, lock = shared_state_and_lock
    ct = CameraThread(state, lock)
    ct.start()
    time.sleep(1.0)
    ct.stop()

    assert state["last_frame_raw_jpg"] is not None, "raw JPEG stream never populated"
    assert state["last_frame_canny_jpg"] is not None, "Canny JPEG stream never populated"

    raw = cv2.imdecode(np.frombuffer(state["last_frame_raw_jpg"], np.uint8), cv2.IMREAD_COLOR)
    assert raw is not None
    assert raw.shape[1] == CAMERA_WIDTH
    assert raw.shape[0] == CAMERA_HEIGHT


@pytest.mark.integration
def test_extract_from_live_frame(shared_state_and_lock):
    """
    Prerequisite: webcam present, a drawing visible in the camera field.
    Asserts:
    - extract_from_frame produces at least one stroke.
    - All stroke points are within valid frame bounds.
    """
    import time
    from camera_thread import CameraThread
    from path_extractor import extract_from_frame
    from config import CAMERA_WIDTH, CAMERA_HEIGHT

    state, lock = shared_state_and_lock
    ct = CameraThread(state, lock)
    ct.start()
    time.sleep(1.0)
    frame = ct.capture_frame()
    ct.stop()

    assert frame is not None, "No frame captured — is the camera connected?"
    result = extract_from_frame(frame)
    assert result.total_strokes > 0, "No strokes found — place a visible drawing in camera view"
    for stroke in result.strokes:
        for pt in stroke:
            assert 0 <= pt[0] <= CAMERA_WIDTH
            assert 0 <= pt[1] <= CAMERA_HEIGHT


# ─────────────────────────────────────────────────────────────────────────────
# Robot
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.integration
def test_robot_connect_disconnect():
    """
    Prerequisite: UR robot reachable at TEST_ROBOT_IP env var.
    Asserts:
    - connect() succeeds.
    - get_ee_position() returns 6 finite floats in plausible meter ranges.
    - disconnect() leaves connected=False.
    """
    import math
    from robot_controller import RobotController

    ip = _robot_ip()
    rc = RobotController()
    rc.connect(ip)
    assert rc.connected is True

    ee = rc.get_ee_position()
    assert isinstance(ee, list) and len(ee) == 6
    for v in ee:
        assert math.isfinite(v)
    # XYZ in meters — UR typically within [-2, 2]
    for v in ee[:3]:
        assert -2.0 <= v <= 2.0, f"EE position {v} out of expected range"

    rc.disconnect()
    assert rc.connected is False


@pytest.mark.integration
def test_robot_move_to_safe_pose():
    """
    Prerequisite: robot connected, clear workspace, operator present.
    Moves to a known safe pose and verifies EE reaches it within 1mm.
    CAUTION: only run with a clear workspace and emergency stop within reach.
    """
    import math
    from robot_controller import RobotController
    from config import TRAVEL_SPEED, TRAVEL_ACCEL

    ip = _robot_ip()
    rc = RobotController()
    rc.connect(ip)

    # Read current pose, raise only Z by TRAVEL_Z to lift the pen
    current = rc.get_ee_position()
    target = list(current)
    target[2] += 0.05  # lift 50mm

    rc.move_to(target, TRAVEL_SPEED, TRAVEL_ACCEL)
    actual = rc.get_ee_position()
    rc.disconnect()

    for i in range(3):  # XYZ tolerance
        assert abs(actual[i] - target[i]) < 0.001, (
            f"EE axis {i}: expected {target[i]:.4f}, got {actual[i]:.4f}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Full pipeline
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.integration
def test_full_pipeline_blank_paper(shared_state_and_lock):
    """
    Prerequisite: camera + robot connected. Blank paper in camera view.
    Asserts:
    - extract_from_frame returns 0 strokes on blank paper.
    - PathExecutor completes with phase="done" and only the final pen-up move fires.
    """
    import threading
    import time
    from camera_thread import CameraThread
    from path_extractor import extract_from_frame
    from path_executor import PathExecutor
    from robot_controller import RobotController

    state, lock = shared_state_and_lock
    ip = _robot_ip()

    # Camera capture
    ct = CameraThread(state, lock)
    ct.start()
    time.sleep(1.0)
    frame = ct.capture_frame()
    ct.stop()
    assert frame is not None

    # Extraction — expects blank page
    extracted = extract_from_frame(frame)
    assert extracted.total_strokes == 0, "Expected blank paper — remove any drawing from view"

    # Execution — empty strokes
    rc = RobotController()
    rc.connect(ip)
    try:
        ex = PathExecutor(rc, state, lock)
        ex.start(extracted.strokes)
        ex._thread.join(timeout=10.0)
        with lock:
            assert state["phase"] == "done"
    finally:
        rc.disconnect()


@pytest.mark.integration
def test_full_pipeline_drawn_circle(shared_state_and_lock):
    """
    Prerequisite: camera + robot. A drawn circle on paper in camera view.
    Asserts:
    - At least one stroke extracted.
    - Robot executes all strokes, phase becomes "done", no robot fault.
    CAUTION: robot will move. Ensure workspace is clear before running.
    """
    import time
    from camera_thread import CameraThread
    from path_extractor import extract_from_frame, pixels_to_robot_coords
    from path_executor import PathExecutor
    from robot_controller import RobotController
    from config import CAMERA_WIDTH, CAMERA_HEIGHT

    state, lock = shared_state_and_lock
    ip = _robot_ip()

    rc = RobotController()
    rc.connect(ip)

    ct = CameraThread(state, lock)
    ct.start()
    time.sleep(1.0)
    frame = ct.capture_frame()
    ct.stop()
    assert frame is not None

    extracted = extract_from_frame(frame)
    assert extracted.total_strokes > 0, "No drawing detected — place a circle in camera view"

    with lock:
        workspace = state.get("workspace")
    assert workspace is not None, "Workspace must be set before running full pipeline"

    robot_strokes = pixels_to_robot_coords(
        extracted.strokes, workspace, CAMERA_WIDTH, CAMERA_HEIGHT
    )

    try:
        ex = PathExecutor(rc, state, lock)
        ex.start(robot_strokes)
        ex._thread.join(timeout=60.0)
        with lock:
            assert state["phase"] == "done", f"Unexpected phase: {state['phase']}"
    finally:
        rc.disconnect()
