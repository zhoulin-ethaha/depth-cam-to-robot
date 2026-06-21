"""
Shared fixtures, mock factories, and synthetic image generators.
Lives at project root so pytest discovers it for all tests/ files.
"""
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest

from workspace import WorkspaceConfig


# ── Synthetic image generators ────────────────────────────────────────────────

@pytest.fixture
def blank_frame():
    """480×640 all-black BGR frame. No edges — Canny returns nothing."""
    return np.zeros((480, 640, 3), dtype=np.uint8)


@pytest.fixture
def frame_with_rectangle():
    """Black frame with a thick white-filled rectangle. Produces clean Canny edges."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (100, 100), (300, 300), (255, 255, 255), -1)
    return frame


@pytest.fixture
def frame_with_tiny_dot():
    """Black frame with a tiny white circle (radius=2). Contour < CONTOUR_MIN_PIXELS."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(frame, (50, 50), 2, (255, 255, 255), -1)
    return frame


@pytest.fixture
def frame_with_two_rectangles():
    """Two white rectangles in opposite quadrants — used for TSP ordering tests."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (50, 50), (150, 150), (255, 255, 255), -1)     # top-left
    cv2.rectangle(frame, (490, 330), (590, 430), (255, 255, 255), -1)   # bottom-right
    return frame


@pytest.fixture
def frame_long_line():
    """Black frame with a single long white horizontal line (200px wide)."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.line(frame, (100, 240), (500, 240), (255, 255, 255), 3)
    return frame


# ── Workspace fixture ─────────────────────────────────────────────────────────

@pytest.fixture
def simple_workspace():
    """
    Axis-aligned workspace for deterministic coordinate mapping:
      origin=[0,0,0], x_axis=[1,0,0], y_axis=[0,1,0], z_axis=[0,0,1]
      x_extent=0.3, y_extent=0.225

    Mapping: pixel (u,v) → robot (u/640*0.3, (1 - v/480)*0.225, 0)
    (v is flipped: image rows grow downward, world Y grows upward)
    """
    return WorkspaceConfig(
        origin=[0.0, 0.0, 0.0],
        x_axis=[1.0, 0.0, 0.0],
        y_axis=[0.0, 1.0, 0.0],
        z_axis=[0.0, 0.0, 1.0],
        x_extent=0.3,
        y_extent=0.225,
    )


# ── Mock robot factory ────────────────────────────────────────────────────────

@pytest.fixture
def mock_robot():
    """
    MagicMock that mirrors RobotController's public interface.
    - connected: True by default
    - get_ee_position(): returns [0.1, 0.2, 0.3, 0.0, 3.14159, 0.0]
    - move_to, stop_motion, start_freedrive, end_freedrive: no-op, call-recording mocks
    """
    robot = MagicMock()
    robot.connected = True
    robot.get_ee_position.return_value = [0.1, 0.2, 0.3, 0.0, 3.14159, 0.0]
    return robot


# ── Shared state factory ──────────────────────────────────────────────────────

@pytest.fixture
def shared_state_and_lock():
    """Returns (state_dict, lock) matching main.py's shared_state structure."""
    state = {
        "robot_connected":      False,
        "last_frame_raw_jpg":   None,
        "last_frame_canny_jpg": None,
        "workspace":            None,
        "pending_workspace":    None,
        "ws_points":            {"p0": None, "px": None, "py": None},
        "freedrive":            False,
        "ee":                   [0.0] * 6,
        "phase":                "idle",
        "strokes":              [],
        "executing":            False,
        "progress":             0.0,
    }
    return state, threading.Lock()
