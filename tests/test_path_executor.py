"""
Unit tests for path_executor.py — PathExecutor with a mock RobotController.
No physical robot required.
"""
import math
import threading
import time

import pytest

from path_executor import PathExecutor
from config import DRAW_Z, TRAVEL_Z, TRAVEL_SPEED, TRAVEL_ACCEL


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

_PI = math.pi

def _make_stroke(n_waypoints: int, x_start=0.1, y=0.2, z=0.0):
    """Helper: make a single stroke with n_waypoints along x."""
    return [
        [x_start + i * 0.01, y, z, 0.0, _PI, 0.0]
        for i in range(n_waypoints)
    ]


def _wait_done(executor, timeout=3.0):
    """Join the executor's background thread."""
    if executor._thread:
        executor._thread.join(timeout=timeout)


# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def executor(mock_robot, shared_state_and_lock):
    state, lock = shared_state_and_lock
    return PathExecutor(mock_robot, state, lock), mock_robot, state


@pytest.fixture
def one_stroke(mock_robot, shared_state_and_lock):
    """Executor + a single stroke with 2 waypoints."""
    state, lock = shared_state_and_lock
    ex = PathExecutor(mock_robot, state, lock)
    strokes = [_make_stroke(2)]  # 2 waypoints
    # EE starts at a known position
    mock_robot.get_ee_position.return_value = [0.0, 0.0, 0.05, 0.0, _PI, 0.0]
    return ex, mock_robot, state, strokes


# ─────────────────────────────────────────────────────────────────────────────
# Lifecycle
# ─────────────────────────────────────────────────────────────────────────────

class TestPathExecutorLifecycle:

    def test_not_running_before_start(self, executor):
        ex, robot, state = executor
        assert ex.running is False

    def test_running_after_start(self, executor):
        ex, robot, state = executor
        ex.start([_make_stroke(1)])
        assert ex.running is True
        _wait_done(ex)

    def test_start_idempotent(self, executor):
        ex, robot, state = executor
        strokes = [_make_stroke(2)]
        ex.start(strokes)
        ex.start(strokes)  # second call must be no-op
        _wait_done(ex)
        # lift + travel + final pen-up = 3 move_to calls; draw goes via servo_to
        assert robot.move_to.call_count == 3
        assert robot.stop_servo.call_count >= 1

    def test_not_running_after_completion(self, executor):
        ex, robot, state = executor
        ex.start([_make_stroke(1)])
        _wait_done(ex)
        assert ex.running is False


# ─────────────────────────────────────────────────────────────────────────────
# Move sequence
# ─────────────────────────────────────────────────────────────────────────────

class TestMoveSequence:

    def test_single_stroke_exact_move_count(self, one_stroke):
        ex, robot, state, strokes = one_stroke
        ex.start(strokes)
        _wait_done(ex)
        # lift + travel + final pen-up = 3 move_to calls; draw goes via servo_to
        assert robot.move_to.call_count == 3
        assert robot.servo_to.call_count >= 1
        assert robot.stop_servo.call_count >= 1

    def test_lift_move_uses_current_ee_z(self, one_stroke):
        ex, robot, state, strokes = one_stroke
        # EE is at z=0.05; lift should go to 0.05 + TRAVEL_Z
        ex.start(strokes)
        _wait_done(ex)
        lift_pose = robot.move_to.call_args_list[0][0][0]
        expected_z = 0.05 + TRAVEL_Z
        assert abs(lift_pose[2] - expected_z) < 1e-9

    def test_travel_move_goes_above_stroke_start(self, one_stroke):
        ex, robot, state, strokes = one_stroke
        ex.start(strokes)
        _wait_done(ex)
        travel_pose = robot.move_to.call_args_list[1][0][0]
        stroke_start = strokes[0][0]
        assert abs(travel_pose[0] - stroke_start[0]) < 1e-9  # x matches
        assert abs(travel_pose[1] - stroke_start[1]) < 1e-9  # y matches
        assert abs(travel_pose[2] - (stroke_start[2] + TRAVEL_Z)) < 1e-9

    def test_draw_moves_apply_draw_z_offset(self, one_stroke):
        ex, robot, state, strokes = one_stroke
        ex.start(strokes)
        _wait_done(ex)
        # All waypoints in _make_stroke have z=0; every servo_to call must use z = DRAW_Z
        for call in robot.servo_to.call_args_list:
            pose = call[0][0]
            assert abs(pose[2] - DRAW_Z) < 1e-9

    def test_lift_and_travel_use_travel_speed(self, one_stroke):
        ex, robot, state, strokes = one_stroke
        ex.start(strokes)
        _wait_done(ex)
        calls = robot.move_to.call_args_list
        # call[0] = lift, call[1] = travel
        for i in [0, 1]:
            _, speed, accel = calls[i][0]
            assert speed == TRAVEL_SPEED
            assert accel == TRAVEL_ACCEL

    def test_draw_calls_servo_for_each_waypoint(self, one_stroke):
        ex, robot, state, strokes = one_stroke
        ex.start(strokes)
        _wait_done(ex)
        # servoL streaming produces at least one call per input waypoint
        assert robot.servo_to.call_count >= len(strokes[0])

    def test_final_pen_up_after_last_stroke(self, one_stroke):
        ex, robot, state, strokes = one_stroke
        final_ee = [0.1, 0.2, 0.05, 0.0, _PI, 0.0]
        # final get_ee_position call returns this
        robot.get_ee_position.side_effect = [
            [0.0, 0.0, 0.05, 0.0, _PI, 0.0],  # first call at start of _run
            final_ee,                            # call at end for final pen-up
        ]
        ex.start(strokes)
        _wait_done(ex)
        last_call = robot.move_to.call_args_list[-1][0][0]
        assert abs(last_call[2] - (final_ee[2] + TRAVEL_Z)) < 1e-9


# ─────────────────────────────────────────────────────────────────────────────
# State transitions
# ─────────────────────────────────────────────────────────────────────────────

class TestStateTransitions:

    def test_state_done_on_success(self, one_stroke):
        ex, robot, state, strokes = one_stroke
        ex.start(strokes)
        _wait_done(ex)
        assert state["phase"] == "done"
        assert state["progress"] == 1.0
        assert state["executing"] is False

    def test_state_captured_on_cancel(self, mock_robot, shared_state_and_lock):
        state, lock = shared_state_and_lock
        # Make move_to block briefly so cancel can fire
        event = threading.Event()
        def slow_move(pose, speed, accel):
            event.wait(timeout=0.5)
        mock_robot.move_to.side_effect = slow_move
        ex = PathExecutor(mock_robot, state, lock)
        ex.start([_make_stroke(3)])
        time.sleep(0.02)
        ex.cancel()
        event.set()
        _wait_done(ex, timeout=3.0)
        assert state["phase"] == "captured"
        assert state["executing"] is False

    def test_cancel_calls_stop_motion(self, mock_robot, shared_state_and_lock):
        state, lock = shared_state_and_lock
        ex = PathExecutor(mock_robot, state, lock)
        ex.start([_make_stroke(3)])
        ex.cancel()
        _wait_done(ex, timeout=2.0)
        assert mock_robot.stop_motion.called

    def test_exception_sets_error_phase(self, mock_robot, shared_state_and_lock):
        state, lock = shared_state_and_lock
        mock_robot.move_to.side_effect = RuntimeError("RTDE disconnected")
        ex = PathExecutor(mock_robot, state, lock)
        ex.start([_make_stroke(2)])
        _wait_done(ex)
        assert state["phase"] == "error"
        assert state["executing"] is False

    def test_progress_fractions(self, mock_robot, shared_state_and_lock):
        """Progress must update after lift, travel, draw, and pen-up phases."""
        state, lock = shared_state_and_lock
        progress_snapshots = []

        def recording_move(pose, speed, accel):
            with lock:
                progress_snapshots.append(state["progress"])

        def recording_servo(pose):
            with lock:
                progress_snapshots.append(state["progress"])

        mock_robot.move_to.side_effect = recording_move
        mock_robot.servo_to.side_effect = recording_servo
        ex = PathExecutor(mock_robot, state, lock)
        strokes = [_make_stroke(2)]
        ex.start(strokes)
        _wait_done(ex)
        # lift + travel (2 move_to) + many servo_to + final pen-up (1 move_to) = many snapshots
        assert len(progress_snapshots) >= 4
        for i in range(1, len(progress_snapshots)):
            assert progress_snapshots[i] >= progress_snapshots[i - 1]

    def test_executing_flag_true_during_run(self, mock_robot, shared_state_and_lock):
        state, lock = shared_state_and_lock
        snapshots = []

        def capture_executing(pose, speed, accel):
            with lock:
                snapshots.append(state["executing"])

        mock_robot.move_to.side_effect = capture_executing
        ex = PathExecutor(mock_robot, state, lock)
        ex.start([_make_stroke(1)])
        _wait_done(ex)
        assert any(snapshots), "executing must be True during _run"


# ─────────────────────────────────────────────────────────────────────────────
# Known bugs / edge cases
# ─────────────────────────────────────────────────────────────────────────────

class TestKnownBehaviors:

    def test_empty_strokes_list_completes_without_draw(self, mock_robot, shared_state_and_lock):
        """
        BUG DOCUMENTATION: empty strokes list is harmless — no draw loop runs,
        only the final pen-up fires. phase becomes "done".
        """
        state, lock = shared_state_and_lock
        ex = PathExecutor(mock_robot, state, lock)
        ex.start([])
        _wait_done(ex)
        assert state["phase"] == "done"
        # Only the final pen-up move_to should have been called
        assert mock_robot.move_to.call_count == 1

    def test_empty_stroke_inner_crashes_to_error_phase(self, mock_robot, shared_state_and_lock):
        """
        BUG DOCUMENTATION: a stroke with zero waypoints causes IndexError on
        stroke[0] inside _run(). The outer try/except catches it and sets phase="error".
        Fix: add `if not stroke: continue` before accessing stroke[0].
        """
        state, lock = shared_state_and_lock
        ex = PathExecutor(mock_robot, state, lock)
        ex.start([[]])  # one stroke, zero waypoints
        _wait_done(ex)
        assert state["phase"] == "error"

    def test_false_progress_when_robot_not_connected(self, mock_robot, shared_state_and_lock):
        """
        BUG DOCUMENTATION: move_to() is a no-op when not connected, but progress
        still increments. This gives a false impression of successful execution.
        Fix: gate progress increment on robot.connected.
        """
        state, lock = shared_state_and_lock
        mock_robot.connected = False
        ex = PathExecutor(mock_robot, state, lock)
        ex.start([_make_stroke(2)])
        _wait_done(ex)
        assert state["phase"] == "done"
        assert state["progress"] == 1.0  # progress=1.0 despite no actual movement

    def test_cz_tracking_uses_stroke_z_plus_draw_z(self, mock_robot, shared_state_and_lock):
        """
        BUG DOCUMENTATION: after each stroke, cz is set to stroke[-1][2] + DRAW_Z,
        NOT from get_ee_position(). This is an approximation that diverges when the
        robot doesn't fully reach the target.
        """
        state, lock = shared_state_and_lock
        # stroke1 ends at z=0.1; next lift should use cz = 0.1 + DRAW_Z, not actual EE
        stroke1 = [[0.1, 0.2, 0.1, 0.0, _PI, 0.0], [0.15, 0.2, 0.1, 0.0, _PI, 0.0]]
        stroke2 = [[0.2, 0.2, 0.0, 0.0, _PI, 0.0]]
        ex = PathExecutor(mock_robot, state, lock)
        ex.start([stroke1, stroke2])
        _wait_done(ex)

        calls = mock_robot.move_to.call_args_list
        # The lift at the start of stroke2 is after: lift(s1) + travel(s1) + move_path(s1) = index 2
        lift_stroke2 = calls[2][0][0]
        expected_cz = stroke1[-1][2] + DRAW_Z  # = 0.1 + (-0.010) = 0.09
        assert abs(lift_stroke2[2] - (expected_cz + TRAVEL_Z)) < 1e-9
