import socket
import threading
from typing import Optional

from config import (
    RTDE_FREQUENCY, SERVO_LOOKAHEAD_TIME, SERVO_GAIN,
    SERVO_VELOCITY_DEFAULT, SERVO_ACCELERATION,
    START_JOINT_ANGLES, START_SPEED, START_ACCEL,
)

_RTDE_PORT = 30004


def _check_rtde_port(ip: str, timeout: float = 3.0) -> None:
    """Fast TCP probe — raises OSError/TimeoutError if port 30004 is unreachable."""
    try:
        with socket.create_connection((ip, _RTDE_PORT), timeout=timeout):
            pass
    except socket.timeout:
        raise TimeoutError(
            f"No response from {ip}:{_RTDE_PORT} after {timeout}s. "
            "Check the robot is powered on and Remote Control is enabled "
            "(Teach pendant → Settings → System → Remote Control)."
        )
    except ConnectionRefusedError:
        raise ConnectionRefusedError(
            f"Port {_RTDE_PORT} refused on {ip}. "
            "The RTDE interface may be disabled or the robot is not in Remote Control mode."
        )
    except OSError as exc:
        raise OSError(f"Cannot reach {ip}:{_RTDE_PORT} — {exc}") from exc


class RobotController:
    """
    Thin wrapper around ur_rtde.
    All public methods are thread-safe via an internal lock.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._rtde_c = None
        self._rtde_r = None
        self._ip: Optional[str] = None

    @property
    def connected(self) -> bool:
        return self._rtde_c is not None

    def connect(self, ip: str) -> None:
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface

        _check_rtde_port(ip)

        rtde_c = RTDEControlInterface(ip)
        rtde_r = RTDEReceiveInterface(ip)

        with self._lock:
            if self._rtde_c is not None:
                self._disconnect_unlocked()
            self._rtde_c = rtde_c
            self._rtde_r = rtde_r
            self._ip = ip

    def disconnect(self) -> None:
        with self._lock:
            self._disconnect_unlocked()

    def _disconnect_unlocked(self) -> None:
        if self._rtde_c:
            try:
                self._rtde_c.stopL(2.0)
            except Exception:
                pass
            try:
                self._rtde_c.servoStop()
            except Exception:
                pass
            try:
                self._rtde_c.disconnect()
            except Exception:
                pass
            self._rtde_c = None
        if self._rtde_r:
            try:
                self._rtde_r.disconnect()
            except Exception:
                pass
            self._rtde_r = None
        self._ip = None

    def move_to_start(self) -> None:
        with self._lock:
            if self._rtde_c is None:
                return
            self._rtde_c.moveJ(START_JOINT_ANGLES, START_SPEED, START_ACCEL)

    def move_to(self, pose_vec: list[float], speed: float, accel: float) -> None:
        """Blocking moveL. Called from the path executor thread."""
        with self._lock:
            if self._rtde_c is None:
                return
            self._rtde_c.moveL(pose_vec, speed, accel)

    def move_path(self, path: list[list[float]]) -> None:
        """Blocking movePath. Each entry: [x,y,z,rx,ry,rz,speed,accel,blend]."""
        with self._lock:
            if self._rtde_c is None:
                return
            self._rtde_c.movePath(path)

    def stop_motion(self) -> None:
        """Stop any in-progress moveL or servoL."""
        with self._lock:
            if self._rtde_c is None:
                return
            try:
                self._rtde_c.stopL(2.0)
            except Exception:
                pass
            try:
                self._rtde_c.servoStop()
            except Exception:
                pass

    def servo_to(self, pose_vec: list[float], velocity: float = SERVO_VELOCITY_DEFAULT) -> None:
        with self._lock:
            if self._rtde_c is None:
                return
            self._rtde_c.servoL(
                pose_vec,
                velocity,
                SERVO_ACCELERATION,
                1.0 / RTDE_FREQUENCY,
                SERVO_LOOKAHEAD_TIME,
                SERVO_GAIN,
            )

    def stop_servo(self) -> None:
        with self._lock:
            if self._rtde_c is None:
                return
            try:
                self._rtde_c.servoStop()
            except Exception:
                pass

    def get_ee_position(self) -> list[float]:
        with self._lock:
            if self._rtde_r is None:
                return [0.0] * 6
            return list(self._rtde_r.getActualTCPPose())

    def start_freedrive(self) -> None:
        with self._lock:
            if self._rtde_c is None:
                return
            self._rtde_c.freedriveMode()

    def end_freedrive(self) -> None:
        with self._lock:
            if self._rtde_c is None:
                return
            try:
                self._rtde_c.endFreedrive()
            except Exception:
                pass
            try:
                self._rtde_c.reuploadScript()
            except Exception:
                pass
