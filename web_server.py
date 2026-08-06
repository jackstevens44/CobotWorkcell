#!/usr/bin/env python3
"""
Local web dashboard for the myCobot 280 M5.

Run:
    python3 web_server.py --port /dev/cu.usbserial-XXXXXXXX

Then open:
    http://127.0.0.1:8765
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import mimetypes
import os
import threading
import time
import urllib.error
import urllib.request
import uuid
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

from mycobot_280_m5_uart import (
    JOINT_LIMITS,
    list_serial_ports,
    validate_joint_angle,
)
from mycobot_driver import MyCobotDriver
from mycobot_kinematics import (
    FIRMWARE_BASE_TRANSLATION_M,
    flange_from_tcp,
    firmware_flange_kinematics,
    grasp_rotation,
    pose_residual,
    rpy_deg_from_rotation,
    rotation_from_rpy_deg,
    solve_pose,
    tcp_from_flange,
    tool_axis_diagnostics,
    top_down_flange_pose,
    top_down_tcp_rotation,
)
from workcell import (
    HOME_ANGLES,
    PHYSICAL_CONFIRM_TOKEN,
    PLANNED_COORDINATE_IK_MARGIN_MM,
    Workcell,
    validate_coordinate_bounds,
)
from camera_service import CameraService
from fiducial_localization import (
    CharucoCalibrationSession,
    ContinuousLocalizationRuntime,
    verification_report,
)
import object_classifier

try:
    from serial import SerialException
except Exception:  # pragma: no cover - pyserial ships with pymycobot
    class SerialException(Exception):
        pass

# Some OS mime databases map .js to text/plain, which makes browsers reject ES modules.
mimetypes.add_type("text/javascript", ".js")
mimetypes.add_type("text/javascript", ".mjs")

ROOT = Path(__file__).resolve().parent
STATIC_ROOT = ROOT / "static"
MAX_REQUEST_BODY_BYTES = 1_000_000
JOINT_TARGET_TOLERANCE_DEG = 5.0
JOINT_TARGET_SOFT_TOLERANCE_DEG = 14.0
JOINT_TARGET_STABLE_SAMPLES = 3
JOINT_SETTLED_DELTA_DEG = 0.8
JOINT_MIN_MOVE_WAIT_S = 0.65
JOINT_FEEDBACK_POLL_S = 0.12
JOINT_MOVE_TIMEOUT_S = 12.0
JOINT_FEEDBACK_MISS_TIMEOUT_S = 2.0


def generated_coordinate_xy_margin_mm(step: Dict[str, Any]) -> float:
    """Allow generated poses to reach full IK; taught/jogged poses stay strict."""
    return (
        PLANNED_COORDINATE_IK_MARGIN_MM
        if isinstance(step.get("targetTcpPoseM"), dict)
        else 0.0
    )
MOTION_FEEDBACK_RECOVERY_WINDOW_S = 5.5
MOTION_FEEDBACK_RECOVERY_DELAY_S = 0.22
MOTION_COMMAND_FEEDBACK_DELAY_S = 0.12
GRIPPER_FEEDBACK_POLL_S = 0.15
GRIPPER_ACTION_TIMEOUT_S = 4.0
PROGRAM_GRIPPER_RECOVERY_TIMEOUT_S = 1.2
PROGRAM_GRIPPER_RECOVERY_DELAY_S = 0.18
PROGRAM_GRIPPER_RECOVERY_ATTEMPTS = 3
PROGRAM_GRIPPER_TIMEOUT_S = 3.0
COORD_TARGET_TOLERANCE_MM = 3.0
COORD_RPY_TOLERANCE_DEG = 3.0
SECURITY_RESPONSE_HEADERS = {
    "Cache-Control": "no-store",
    "Cross-Origin-Opener-Policy": "same-origin",
    "Cross-Origin-Resource-Policy": "same-origin",
    "Permissions-Policy": "camera=(self), microphone=(self)",
    "Referrer-Policy": "no-referrer",
    "X-Content-Type-Options": "nosniff",
    "X-Frame-Options": "DENY",
}


def is_loopback_bind_host(host: Any) -> bool:
    """Return true only for an explicit local-only HTTP bind address."""
    candidate = str(host or "").strip().lower().rstrip(".")
    if candidate == "localhost":
        return True
    try:
        address = ipaddress.ip_address(candidate)
        return isinstance(address, ipaddress.IPv4Address) and address.is_loopback
    except ValueError:
        return False


def json_safe(value: Any, _active: Optional[set] = None, _depth: int = 0) -> Any:
    """Return a strict-JSON-compatible copy without recursing on cycles."""
    if _depth > 64:
        return None
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (dict, list, tuple)):
        active = _active if _active is not None else set()
        identity = id(value)
        if identity in active:
            return None
        active.add(identity)
        try:
            if isinstance(value, dict):
                return {key: json_safe(item, active, _depth + 1) for key, item in value.items()}
            return [json_safe(item, active, _depth + 1) for item in value]
        finally:
            active.remove(identity)
    return value
# Real servo/controller endpoints are less exact than firmware IK/FK math.
# Physical traces show pose-dependent get_coords() offsets up to 13.3 mm even
# after a completed, stable move. Fifteen millimeters is also the largest miss
# that preserves the planner's 4 mm minimum fingertip clearance at the bin wall.
# The same 15 mm / 5 degree values are the hard disagreement envelope for the
# host and firmware models. The 3 mm / 3 degree values remain the precision
# target and produce explicit warnings before that hard boundary.
COORD_PHYSICAL_TOLERANCE_MM = 15.0
COORD_PHYSICAL_ANGULAR_TOLERANCE_MM = 15.0
COORD_PHYSICAL_RPY_TOLERANCE_DEG = 5.0
# ``solve_inv_kinematics`` followed by ``angles_to_coords`` is an approximate
# controller-side preview, not the closed-loop result of ``send_coords``.  The
# real arm has repeatedly completed coordinate moves whose preview round trip
# was 5-13 mm away even though the requested Cartesian target itself was valid.
# Keep 3 mm as the precision target and surface anything above it as a warning,
# but only reject the preview at the same conservative boundary used by the
# post-motion coordinate verifier.
COORD_IK_ROUNDTRIP_TOLERANCE_MM = COORD_PHYSICAL_TOLERANCE_MM
COORD_IK_ROUNDTRIP_RPY_TOLERANCE_DEG = COORD_PHYSICAL_RPY_TOLERANCE_DEG
COORD_STOP_STABLE_SAMPLES = 3
COORD_SETTLED_DELTA_MM = 1.0
IK_PREVIEW_MAX_JOINT_STEP_DEG = 75.0
# A top-down jaw yaw can legitimately require J6 to rotate 35-60 degrees even
# when J1 does not move by the same amount.  Larger wrist flips remain rejected
# independently of the general 75-degree discontinuity limit.
IK_PREVIEW_MAX_J6_STEP_DEG = 60.0
SUCTION_J6_LOCK_TOLERANCE_DEG = 0.5
SUCTION_J6_EXECUTION_TOLERANCE_DEG = 1.0
IK_PREVIEW_MIN_JOINT_MARGIN_DEG = 0.0
IK_ORIENTATION_YAW_OFFSETS_DEG = (0.0, 180.0)
# Once a part is securely held and the destination is a bin, its yaw may be
# changed during the angular carry.  The selected yaw is then held constant for
# the complete carry/lower/retreat group.  This avoids rejecting reachable bin
# drops merely because the pick-aligned wrist yaw is singular at the bin.
IK_FREE_CARRY_YAW_OFFSETS_DEG = (0.0, 180.0, 90.0, -90.0, 45.0, -45.0, 135.0, -135.0)
IK_ORIENTATION_TILT_OFFSETS_DEG = (0.0, 2.0, 4.0, 6.0, 8.0, 10.0)
MAX_TOP_DOWN_TILT_DEG = 10.0
HOST_FK_HARD_POSITION_TOLERANCE_MM = 15.0
HOST_FK_HARD_ORIENTATION_TOLERANCE_DEG = 5.0
INWARD_REACH_SEARCH_STEP_MM = 5.0
INWARD_REACH_SEARCH_MAX_MM = 120.0
CONTROLLER_ERROR_LABELS = {
    0: "no_controller_error",
    1: "joint_1_limit",
    2: "joint_2_limit",
    3: "joint_3_limit",
    4: "joint_4_limit",
    5: "joint_5_limit",
    6: "joint_6_limit",
    16: "collision_protection_1",
    17: "collision_protection_2",
    18: "collision_protection_3",
    19: "collision_protection_4",
    32: "controller_ik_no_solution",
    33: "controller_linear_no_adjacent_solution",
    34: "controller_linear_no_adjacent_solution",
}
MAX_PLANNED_JAW_CENTER_ERROR_MM = 1.0
MAX_IK_JAW_CENTER_ERROR_MM = COORD_IK_ROUNDTRIP_TOLERANCE_MM


class HostKinematicsPreviewRobot:
    """Read-only offline stand-in for firmware IK/FK preview calls."""

    def __init__(
        self, tool_id: str = "adaptive_gripper",
        correction_local_m: Optional[List[float]] = None,
        suction_contact_distance_m: float = 0.072,
    ) -> None:
        self.tool_id = tool_id
        self.correction_local_m = correction_local_m or [0.0, 0.0, 0.0]
        self.suction_contact_distance_m = float(suction_contact_distance_m)
        # Candidate ranking owns the bounded exhaustive fallback. Individual
        # offline firmware stand-in calls stay on the fast deterministic seeds.
        self.exhaustive = False

    def solve_inv_kinematics(self, coords: List[float], current: List[float]) -> List[float]:
        flange_position = tuple(float(value) / 1000.0 for value in coords[:3])
        flange_rotation = rotation_from_rpy_deg(coords[3:6])
        tcp_position, tcp_rotation = tcp_from_flange(
            flange_position, flange_rotation, self.tool_id,
            self.correction_local_m, self.suction_contact_distance_m,
        )
        model_tcp_position = tuple(
            tcp_position[index] - FIRMWARE_BASE_TRANSLATION_M[index] for index in range(3)
        )
        solution = solve_pose(
            model_tcp_position, tcp_rotation, current,
            exhaustive=self.exhaustive,
            tool_id=self.tool_id,
            correction_local_m=self.correction_local_m,
            suction_contact_distance_m=self.suction_contact_distance_m,
        )
        if solution is None:
            raise ValueError("host_ik_unreachable")
        return solution

    @staticmethod
    def angles_to_coords(angles: List[float]) -> List[float]:
        position, rotation = firmware_flange_kinematics(angles)
        return [
            position[0] * 1000.0,
            position[1] * 1000.0,
            position[2] * 1000.0,
            *rpy_deg_from_rotation(rotation),
        ]


def load_env_file(path: Path) -> None:
    if not path.exists():
        return
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


# Local secrets live in api_keys.env so API keys can stay out of Git. Keep the
# older .env path as a fallback for existing local setups.
load_env_file(ROOT / "api_keys.env")
load_env_file(ROOT / ".env")

REALTIME_MODEL = os.environ.get("OPENAI_REALTIME_MODEL", "gpt-realtime-2.1")
REALTIME_VOICE = os.environ.get("OPENAI_REALTIME_VOICE", "marin")
REALTIME_PLAN_TTL_S = 20 * 60
REALTIME_RUN_CONFIRM_TTL_S = 45


class PlanAborted(RuntimeError):
    """Raised when the operator stops a running physical plan."""


class MotionProgressError(RuntimeError):
    """Raised when a motion state stops making useful feedback progress."""

    def __init__(
        self,
        reason: str,
        target: List[float],
        actual: List[float],
        error_deg: float,
        message: str,
        feedback_misses: int = 0,
        details: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(message)
        self.reason = reason
        self.target = target
        self.actual = actual
        self.error_deg = error_deg
        self.feedback_misses = feedback_misses
        self.details = details or {}

    def result(self) -> Dict[str, Any]:
        result = {
            "command": "failed_motion",
            "failureReason": self.reason,
            "targetAngles": [round(value, 2) for value in self.target],
            "actualAngles": [round(value, 2) for value in self.actual],
            "errorDeg": round(self.error_deg, 2),
            "feedbackMisses": self.feedback_misses,
        }
        result.update(self.details)
        return result


class CoordinateMotionError(RuntimeError):
    """Raised when a firmware coordinate motion cannot be verified safely."""

    def __init__(
        self,
        reason: str,
        target_coords: List[float],
        actual_coords: Optional[List[float]],
        message: str,
        details: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(message)
        self.reason = reason
        self.target_coords = target_coords
        self.actual_coords = actual_coords
        self.details = details or {}

    def result(self) -> Dict[str, Any]:
        result = {
            "command": "send_coords",
            "failureReason": self.reason,
            "targetCoords": [round(value, 2) for value in self.target_coords],
        }
        if self.actual_coords is not None:
            result["actualCoords"] = [round(value, 2) for value in self.actual_coords]
        result.update(self.details)
        return result


class RobotService:
    def __init__(self, port: Optional[str], baud: int, timeout: float) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.robot: Optional[MyCobotDriver] = None
        self.lock = threading.Lock()
        self.last_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_coords: Optional[List[float]] = None
        self.last_error: Optional[str] = None
        self.last_read_at: Optional[float] = None
        self.executing = False
        self.execution_progress: Optional[Dict[str, Any]] = None
        self.abort_event = threading.Event()
        self.end_effector = "adaptive_gripper"
        self.tool_profile: Dict[str, Any] = {}
        self.jog_session: Optional[Dict[str, Any]] = None
        self._jog_watchdog_thread = threading.Thread(
            target=self._jog_watchdog_loop,
            name="mycobot-jog-watchdog",
            daemon=True,
        )
        self._jog_watchdog_thread.start()

    def set_end_effector(self, value: Any) -> None:
        selected = str(value or "adaptive_gripper")
        self.end_effector = selected if selected in ("adaptive_gripper", "suction_gripper") else "adaptive_gripper"

    def set_tool_profile(self, profile: Any) -> None:
        self.tool_profile = deepcopy(profile) if isinstance(profile, dict) else {}

    def _tool_parameters(self) -> Tuple[List[float], float]:
        correction = self.tool_profile.get("tcpCorrectionLocalM") or {}
        geometry = self.tool_profile.get("geometry") or {}
        return (
            [float(correction.get(axis, 0.0)) for axis in ("x", "y", "z")],
            float(geometry.get("flangeToContactM", 0.072)),
        )

    def configure(self, port: Optional[str] = None, baud: Optional[int] = None) -> Dict[str, Any]:
        with self.lock:
            if port is not None and port != self.port:
                self.close_locked()
                self.port = port
            if baud is not None and baud != self.baud:
                self.close_locked()
                self.baud = baud
            return self.status_locked()

    def status(self) -> Dict[str, Any]:
        with self.lock:
            return self.status_locked()

    def status_locked(self) -> Dict[str, Any]:
        return {
            "connected": self.robot is not None,
            "port": self.port,
            "baud": self.baud,
            "lastAngles": self.last_angles,
            "lastCoords": self.last_coords,
            "lastError": self.last_error,
            "lastReadAt": self.last_read_at,
            "executing": self.executing,
            "executionProgress": deepcopy(self.execution_progress),
            "jogging": self.jog_session is not None,
            "jog": deepcopy(self.jog_session) if self.jog_session else None,
            "endEffector": self.end_effector,
            "jointLimits": JOINT_LIMITS,
        }

    def close_locked(self) -> None:
        self._stop_jog_locked("connection_closed")
        if self.robot is not None:
            self.robot.close()
            self.robot = None

    def _stop_jog_locked(self, reason: str = "stopped") -> None:
        session = self.jog_session
        self.jog_session = None
        if session and self.robot is not None:
            try:
                self.robot.jog_stop()
            except Exception:
                try:
                    self.robot.stop()
                except Exception:
                    pass

    def _jog_watchdog_loop(self) -> None:
        while True:
            time.sleep(0.05)
            with self.lock:
                session = self.jog_session
                if not session:
                    continue
                now = time.monotonic()
                reason = None
                if now > float(session.get("heartbeatDeadline") or 0.0):
                    reason = "heartbeat_timeout"
                elif now > float(session.get("maximumDeadline") or 0.0):
                    reason = "maximum_hold_reached"
                else:
                    joint_id = int(session.get("jointId") or 0)
                    if 1 <= joint_id <= 6 and len(self.last_angles) >= joint_id:
                        low, high = JOINT_LIMITS[joint_id]
                        angle = float(self.last_angles[joint_id - 1])
                        if (
                            int(session.get("direction") or 0) == 0
                            and angle <= low + 1.0
                        ) or (
                            int(session.get("direction") or 0) == 1
                            and angle >= high - 1.0
                        ):
                            reason = "joint_limit_guard"
                if reason:
                    self._stop_jog_locked(reason)

    @staticmethod
    def _jog_speed(value: Any) -> int:
        try:
            speed = int(value)
        except (TypeError, ValueError):
            raise ValueError("Jog speed must be a number from 1 to 30.")
        if speed < 1 or speed > 30:
            raise ValueError("Jog speed must be between 1 and 30.")
        return speed

    def start_joint_jog(self, joint_id: Any, direction: Any, speed: Any) -> Dict[str, Any]:
        try:
            joint = int(joint_id)
            direction_value = int(direction)
            speed_value = self._jog_speed(speed)
        except (TypeError, ValueError) as exc:
            return {"ok": False, "error": str(exc), **self.status()}
        if joint < 1 or joint > 6:
            return {"ok": False, "error": "Joint ID must be between 1 and 6.", **self.status()}
        if direction_value not in (0, 1):
            return {"ok": False, "error": "Jog direction must be 0 (negative) or 1 (positive).", **self.status()}
        if self.end_effector == "suction_gripper" and joint == 6:
            return {"ok": False, "error": "J6 jogging is locked while the suction tool is active.", **self.status()}
        with self.lock:
            if self.executing:
                return {"ok": False, "error": "A physical program is running; press Stop first.", **self.status_locked()}
            try:
                robot = self.get_robot_locked()
                self._stop_jog_locked("replaced")
                robot.jog_angle(joint, direction_value, speed_value)
                now = time.monotonic()
                session_id = uuid.uuid4().hex
                self.jog_session = {
                    "sessionId": session_id,
                    "jointId": joint,
                    "direction": direction_value,
                    "speed": speed_value,
                    "startedAt": time.time(),
                    "heartbeatDeadline": now + 0.6,
                    "maximumDeadline": now + 10.0,
                }
                return {"ok": True, "jog": deepcopy(self.jog_session), **self.status_locked()}
            except Exception as exc:
                self._stop_jog_locked("start_failed")
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}

    def heartbeat_jog(self, session_id: Any) -> Dict[str, Any]:
        with self.lock:
            if not self.jog_session:
                return {"ok": False, "error": "No joint jog is active.", **self.status_locked()}
            if str(session_id or "") != str(self.jog_session.get("sessionId")):
                return {"ok": False, "error": "This jog session is no longer active.", **self.status_locked()}
            self.jog_session["heartbeatDeadline"] = time.monotonic() + 0.6
            return {"ok": True, **self.status_locked()}

    def stop_jog(self) -> Dict[str, Any]:
        with self.lock:
            self._stop_jog_locked("operator_stop")
            return {"ok": True, "jogging": False, **self.status_locked()}

    def step_jog(
        self, space: Any, axis_id: Any, increment: Any, speed: Any
    ) -> Dict[str, Any]:
        try:
            selected_space = str(space or "joint").lower()
            axis = int(axis_id)
            increment_value = float(increment)
            speed_value = self._jog_speed(speed)
        except (TypeError, ValueError) as exc:
            return {"ok": False, "error": str(exc), **self.status()}
        if selected_space not in ("joint", "tcp"):
            return {"ok": False, "error": "Jog space must be 'joint' or 'tcp'.", **self.status()}
        if axis < 1 or axis > 6 or not math.isfinite(increment_value) or increment_value == 0:
            return {"ok": False, "error": "Jog axis must be 1–6 with a finite non-zero increment.", **self.status()}
        maximum = 5.0 if selected_space == "joint" or axis >= 4 else 10.0
        if abs(increment_value) > maximum:
            unit = "degrees" if selected_space == "joint" or axis >= 4 else "millimeters"
            return {"ok": False, "error": f"One jog step cannot exceed {maximum:g} {unit}.", **self.status()}
        if selected_space == "joint" and self.end_effector == "suction_gripper" and axis == 6:
            return {"ok": False, "error": "J6 jogging is locked while the suction tool is active.", **self.status()}
        with self.lock:
            if self.executing or self.jog_session:
                return {"ok": False, "error": "Stop the current motion before stepping another axis.", **self.status_locked()}
            try:
                robot = self.get_robot_locked()
                if selected_space == "joint":
                    current = self.read_angles_locked(robot)
                    target = float(current[axis - 1]) + increment_value
                    validate_joint_angle(axis, target)
                    robot.jog_increment_angle(axis, increment_value, speed_value)
                elif self.end_effector == "suction_gripper":
                    current_angles = self.read_angles_locked(robot)
                    current_coords = self.read_coords_locked(robot)
                    target_coords = list(current_coords)
                    target_coords[axis - 1] += increment_value
                    bounds = validate_coordinate_bounds(target_coords, "tcp_jog", allow_missing_rpy=False)
                    if bounds:
                        raise ValueError(bounds[0].get("message") or "TCP jog target is outside coordinate bounds.")
                    solved = robot.solve_inv_kinematics(target_coords, current_angles)
                    solved[5] = current_angles[5]
                    locked_fk = robot.angles_to_coords(solved)
                    errors = self.coords_error(target_coords, locked_fk, 5.0, 3.0)
                    if not errors.get("withinTolerance"):
                        raise ValueError("That TCP nudge cannot preserve the suction J6 lock.")
                    robot.send_angles(solved, speed_value)
                else:
                    robot.jog_increment_coord(axis, increment_value, speed_value)
                return {
                    "ok": True,
                    "space": selected_space,
                    "axisId": axis,
                    "increment": increment_value,
                    "speed": speed_value,
                    **self.status_locked(),
                }
            except Exception as exc:
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}

    def get_robot_locked(self) -> MyCobotDriver:
        if not self.port:
            raise RuntimeError("No serial port selected")
        if self.robot is None:
            try:
                self.robot = MyCobotDriver(self.port, baud=self.baud, timeout=self.timeout)
            except Exception:
                # One reconnect attempt before surfacing the failure.
                time.sleep(0.3)
                self.robot = MyCobotDriver(self.port, baud=self.baud, timeout=self.timeout)
        return self.robot

    def get_angles(self) -> Dict[str, Any]:
        with self.lock:
            if self.executing:
                # The running plan polls feedback constantly, so the cache is fresh.
                return {"ok": True, "angles": self.last_angles, "cached": True, **self.status_locked()}
            try:
                angles = self.get_robot_locked().get_angles()
                self.last_angles = angles
                self.last_error = None
                self.last_read_at = time.time()
                return {"ok": True, "angles": angles, **self.status_locked()}
            except SerialException as exc:
                # Fatal link error: drop the port so the next call reconnects.
                self.close_locked()
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}
            except Exception as exc:
                # Transient read miss: keep the port; the next call retries.
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}

    def get_coords(self) -> Dict[str, Any]:
        with self.lock:
            if self.executing:
                return {"ok": True, "coords": self.last_coords, "cached": True, **self.status_locked()}
            try:
                coords = self.get_robot_locked().get_coords()
                self.last_coords = coords
                self.last_error = None
                self.last_read_at = time.time()
                return {"ok": True, "coords": coords, **self.status_locked()}
            except SerialException as exc:
                self.close_locked()
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}
            except Exception as exc:
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}

    def kinematics_snapshot(self) -> Dict[str, Any]:
        """Read-only comparison of firmware coordinates with host flange FK."""
        with self.lock:
            if self.executing:
                return {"ok": False, "error": "Frame calibration requires a stationary robot."}
            try:
                robot = self.get_robot_locked()
                angles = self.read_angles_locked(robot)
                firmware_coords = self.read_coords_locked(robot)
            except Exception as exc:
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}
        host_pos, host_rot = firmware_flange_kinematics(angles)
        target_pos = tuple(float(value) / 1000.0 for value in firmware_coords[:3])
        target_rot = rotation_from_rpy_deg(firmware_coords[3:6])
        pos_error, rot_error = pose_residual(host_pos, host_rot, target_pos, target_rot)
        host_rpy = self._rotation_to_rpy_deg(host_rot)
        return {
            "ok": True,
            "stationaryReadOnly": True,
            "capturedAt": time.time(),
            "anglesDeg": [round(value, 3) for value in angles],
            "firmwareFlangeCoords": [round(value, 3) for value in firmware_coords],
            "hostFlangeCoords": [
                round(host_pos[0] * 1000.0, 3), round(host_pos[1] * 1000.0, 3),
                round(host_pos[2] * 1000.0, 3), *[round(value, 3) for value in host_rpy],
            ],
            "positionErrorMm": round(pos_error * 1000.0, 3),
            "orientationErrorDeg": round(math.degrees(rot_error), 3),
            "withinTolerance": pos_error * 1000.0 <= COORD_TARGET_TOLERANCE_MM and math.degrees(rot_error) <= COORD_RPY_TOLERANCE_DEG,
        }

    @staticmethod
    def _rotation_to_rpy_deg(rotation: Any) -> List[float]:
        from mycobot_kinematics import rpy_deg_from_rotation
        return list(rpy_deg_from_rotation(rotation))

    def add_coordinate_preview(self, plan: Dict[str, Any], start_angles: List[float]) -> Dict[str, Any]:
        if not plan.get("ok") or plan.get("mode") != "coordinate_program":
            return plan
        preview = {
            "ok": False,
            "source": "pymycobot_solve_inv_kinematics" if self.port else "host_offline_kinematics",
            "solvedStates": 0,
            "error": None,
            "planningDiagnostics": {
                "orientationCandidates": 0,
                "fastHostSolves": 0,
                "exhaustiveHostSolves": 0,
                "hostCacheHits": 0,
                "firmwareIkCalls": 0,
                "exhaustiveFallbackCandidates": 0,
            },
        }

        current = [float(value) for value in (start_angles or [0.0] * 6)[:6]]
        try:
            with self.lock:
                correction, suction_distance = self._tool_parameters()
                robot = self.get_robot_locked() if self.port else HostKinematicsPreviewRobot(
                    self.end_effector, correction, suction_distance
                )
                all_steps = plan.get("steps") or []
                required_states = sum(
                    1 for step in all_steps
                    if isinstance(step.get("coordsMm"), list)
                    or isinstance(step.get("jointTargetDeg"), list)
                )

                def coordinate_group_key(step: Dict[str, Any]) -> str:
                    state_id = str(step.get("stateId") or step.get("name") or "unknown")
                    prefix, _, suffix = state_id.partition("_s")
                    if step.get("orientationPolicy") == "fixed_taught_pose":
                        return f"{prefix}:fixed_taught_pose"
                    try:
                        state_number = int(suffix.split("_", 1)[0])
                    except (TypeError, ValueError):
                        state_number = 0
                    phase = "pick" if state_number and state_number <= 4 else "place"
                    return f"{prefix}:{phase}"

                state_results: List[Dict[str, Any]] = []
                cursor = 0
                while cursor < len(all_steps):
                    step = all_steps[cursor]
                    if step.get("robotCommand") == "home":
                        current = [float(value) for value in HOME_ANGLES]
                        step["previewAngles"] = [round(value, 2) for value in HOME_ANGLES]
                        cursor += 1
                        continue
                    if isinstance(step.get("jointTargetDeg"), list):
                        state_id = str(step.get("stateId") or step.get("name") or "joint_move")
                        try:
                            target = [float(value) for value in step["jointTargetDeg"]]
                        except (TypeError, ValueError):
                            target = []
                        reasons: List[str] = []
                        if len(target) != 6 or not all(math.isfinite(value) for value in target):
                            reasons.append("joint_target_invalid")
                        else:
                            for joint, value in enumerate(target, 1):
                                low, high = JOINT_LIMITS[joint]
                                if value < low or value > high:
                                    reasons.append(f"joint_{joint}_outside_limits")
                            # An explicit Joint Move intentionally permits a
                            # large endpoint change. Validate the firmware's
                            # straight joint interpolation rather than applying
                            # the IK-solution discontinuity threshold used for
                            # adjacent Cartesian waypoints.
                            joint_deltas = [target[i] - current[i] for i in range(6)]
                            joint_steps = [abs(value) for value in joint_deltas]
                            sample_count = max(1, int(math.ceil(max(joint_steps) / 5.0)))
                            trajectory = []
                            for sample_index in range(1, sample_count + 1):
                                fraction = sample_index / sample_count
                                sample = [
                                    current[i] + joint_deltas[i] * fraction
                                    for i in range(6)
                                ]
                                for joint, value in enumerate(sample, 1):
                                    low, high = JOINT_LIMITS[joint]
                                    if value < low or value > high:
                                        reasons.append(f"joint_{joint}_interpolation_outside_limits")
                                trajectory.append({
                                    "t": round(fraction, 4),
                                    "angles": [round(value, 3) for value in sample],
                                })
                        result_state = {
                            "ok": not reasons,
                            "stateId": state_id,
                            "angles": target,
                            "rejectionReasons": reasons,
                            "maxJointStepDeg": round(max(joint_steps), 2) if len(target) == 6 else None,
                        }
                        captured = step.get("capturedFlangeCoordsMmDeg")
                        if not reasons and isinstance(captured, list) and len(captured) == 6:
                            actual_position, actual_rotation = firmware_flange_kinematics(target)
                            expected_position = tuple(float(value) / 1000.0 for value in captured[:3])
                            expected_rotation = rotation_from_rpy_deg(captured[3:6])
                            position_error, rotation_error = pose_residual(
                                actual_position, actual_rotation, expected_position, expected_rotation
                            )
                            result_state["hostPositionErrorMm"] = round(position_error * 1000.0, 3)
                            result_state["hostOrientationErrorDeg"] = round(math.degrees(rotation_error), 3)
                            if position_error * 1000.0 > HOST_FK_HARD_POSITION_TOLERANCE_MM:
                                reasons.append("captured_joint_fk_position_residual")
                            if math.degrees(rotation_error) > HOST_FK_HARD_ORIENTATION_TOLERANCE_DEG:
                                reasons.append("captured_joint_fk_orientation_residual")
                            result_state["ok"] = not reasons
                        state_results.append(result_state)
                        if reasons:
                            preview["error"] = f"{state_id} failed joint-space validation."
                            break
                        step["previewAngles"] = [round(value, 3) for value in target]
                        step["trajectory"] = trajectory
                        current = target
                        preview["solvedStates"] += 1
                        cursor += 1
                        continue
                    if isinstance(step.get("coordsMm"), list):
                        group_key = coordinate_group_key(step)
                        group_steps = [step]
                        lookahead = cursor + 1
                        while (
                            lookahead < len(all_steps)
                            and isinstance(all_steps[lookahead].get("coordsMm"), list)
                            and coordinate_group_key(all_steps[lookahead]) == group_key
                        ):
                            group_steps.append(all_steps[lookahead])
                            lookahead += 1
                        result = self._preview_coordinate_group(robot, group_steps, current)
                        for key, value in (result.get("planningDiagnostics") or {}).items():
                            if isinstance(value, (int, float)):
                                preview["planningDiagnostics"][key] = (
                                    preview["planningDiagnostics"].get(key, 0) + value
                                )
                        state_results.extend(result["states"])
                        if not result["ok"]:
                            preview["error"] = result["error"]
                            if result.get("suggestedInwardShiftMm") is not None:
                                preview["suggestedInwardShiftMm"] = result["suggestedInwardShiftMm"]
                            if result.get("correctiveGuidance"):
                                preview["correctiveGuidance"] = result["correctiveGuidance"]
                            break
                        current = result["endAngles"]
                        preview["solvedStates"] += len(group_steps)
                        cursor = lookahead
                        continue
                    cursor += 1
                preview["states"] = state_results
                preview["requiredStates"] = required_states
            preview["ok"] = (
                preview["solvedStates"] == preview.get("requiredStates", 0)
                and preview.get("error") is None
            )
            if preview.get("requiredStates") == 0:
                preview["coordinateValidationRequired"] = False
                preview["jointOnlyPlan"] = True
        except Exception as exc:
            preview["error"] = str(exc)
        plan["coordinatePreview"] = preview
        plan["planningDiagnostics"] = deepcopy(preview.get("planningDiagnostics") or {})
        plan["physicalReady"] = bool(plan.get("physicalReady", True) and preview["ok"])
        return plan

    @staticmethod
    def _joint_solution_diagnostics(angles: Any, previous: List[float]) -> Dict[str, Any]:
        reasons: List[str] = []
        if not isinstance(angles, (list, tuple)) or len(angles) < 6:
            return {"ok": False, "rejectionReasons": ["firmware_result_not_six_angles"]}
        values = [float(value) for value in angles[:6]]
        if not all(math.isfinite(value) for value in values):
            return {"ok": False, "rejectionReasons": ["firmware_result_non_finite"]}
        if max(values) - min(values) < 0.01:
            return {"ok": False, "rejectionReasons": ["firmware_repeated_angle_failure_pattern"], "angles": values}
        margins = []
        for index, value in enumerate(values, 1):
            low, high = JOINT_LIMITS[index]
            margin = min(value - low, high - value)
            margins.append(margin)
            if margin < IK_PREVIEW_MIN_JOINT_MARGIN_DEG:
                reasons.append(f"joint_{index}_outside_limits")
        signed_steps = [((value - float(previous[i]) + 180.0) % 360.0) - 180.0 for i, value in enumerate(values)]
        steps = [abs(value) for value in signed_steps]
        max_step = max(steps)
        if max_step > IK_PREVIEW_MAX_JOINT_STEP_DEG:
            reasons.append("joint_discontinuity")
        base_wrist_coupled = abs(steps[5] - steps[0]) <= 5.0
        if steps[5] > IK_PREVIEW_MAX_J6_STEP_DEG and not base_wrist_coupled:
            reasons.append("unnecessary_joint_6_rotation")
        return {
            "ok": not reasons,
            "angles": values,
            "minJointLimitMarginDeg": round(min(margins), 2),
            "maxJointStepDeg": round(max_step, 2),
            "jointStepsDeg": [round(value, 2) for value in steps],
            "joint6StepDeg": round(steps[5], 2),
            "baseWristCoupled": base_wrist_coupled,
            "rejectionReasons": reasons,
        }

    @classmethod
    def _validate_firmware_ik(
        cls, coords: List[float], angles: Any, previous: List[float],
        firmware_fk_coords: Optional[List[float]] = None,
        tool_id: str = "adaptive_gripper",
        correction_local_m: Optional[List[float]] = None,
        suction_contact_distance_m: float = 0.072,
        host_solution_override: Optional[List[float]] = None,
    ) -> Dict[str, Any]:
        result = cls._joint_solution_diagnostics(angles, previous)
        if not result.get("angles"):
            return result
        actual_pos, actual_rot = firmware_flange_kinematics(result["angles"])
        target_pos = tuple(float(value) / 1000.0 for value in coords[:3])
        target_rot = rotation_from_rpy_deg(coords[3:6])
        pos_error, rot_error = pose_residual(actual_pos, actual_rot, target_pos, target_rot)
        host_position_error_mm = pos_error * 1000.0
        host_orientation_error_deg = math.degrees(rot_error)
        result["hostPositionErrorMm"] = round(host_position_error_mm, 3)
        result["hostOrientationErrorDeg"] = round(host_orientation_error_deg, 3)

        # The firmware IK/FK pair is not independent: an invalid controller
        # branch can round-trip through both calls and still be refused by
        # send_coords. Require the calibrated host model to agree with the
        # firmware joint solution inside a conservative physical envelope.
        if host_position_error_mm > HOST_FK_HARD_POSITION_TOLERANCE_MM:
            result["rejectionReasons"].append("host_fk_position_residual")
        if host_orientation_error_deg > HOST_FK_HARD_ORIENTATION_TOLERANCE_DEG:
            result["rejectionReasons"].append("host_fk_orientation_residual")
        if (
            host_position_error_mm > COORD_TARGET_TOLERANCE_MM
            or host_orientation_error_deg > COORD_RPY_TOLERANCE_DEG
        ):
            result.setdefault("accuracyWarnings", []).append(
                "host_fk_above_precision_target"
            )

        # Independently solve the same physical TCP pose with the host model.
        # Firmware coordinates include a measured base translation that the
        # host model does not, so remove it before solving.
        target_tcp_position, target_tcp_rotation = tcp_from_flange(
            target_pos, target_rot, tool_id, correction_local_m,
            suction_contact_distance_m,
        )
        model_tcp_position = tuple(
            target_tcp_position[index] - FIRMWARE_BASE_TRANSLATION_M[index]
            for index in range(3)
        )
        host_solution = host_solution_override or solve_pose(
            model_tcp_position, target_tcp_rotation, result["angles"], exhaustive=False,
            tool_id=tool_id, correction_local_m=correction_local_m,
            suction_contact_distance_m=suction_contact_distance_m,
        )
        result["hostIkReachable"] = host_solution is not None
        if host_solution is None:
            result["rejectionReasons"].append("host_ik_unreachable")
        else:
            host_flange_position, host_flange_rotation = firmware_flange_kinematics(host_solution)
            host_solve_pos_error, host_solve_rot_error = pose_residual(
                host_flange_position, host_flange_rotation, target_pos, target_rot
            )
            result["hostIkAngles"] = [round(float(value), 3) for value in host_solution]
            result["hostIkPositionErrorMm"] = round(host_solve_pos_error * 1000.0, 3)
            result["hostIkOrientationErrorDeg"] = round(math.degrees(host_solve_rot_error), 3)
            if host_solve_pos_error * 1000.0 > COORD_TARGET_TOLERANCE_MM:
                result["rejectionReasons"].append("host_ik_position_residual")
            if math.degrees(host_solve_rot_error) > COORD_RPY_TOLERANCE_DEG:
                result["rejectionReasons"].append("host_ik_orientation_residual")
        if firmware_fk_coords is not None:
            firmware_errors = cls.coords_error(
                coords,
                firmware_fk_coords,
                COORD_IK_ROUNDTRIP_TOLERANCE_MM,
                COORD_IK_ROUNDTRIP_RPY_TOLERANCE_DEG,
            )
            result["firmwareFkCoords"] = [round(float(value), 3) for value in firmware_fk_coords]
            result["positionErrorMm"] = firmware_errors["maxPositionErrorMm"]
            result["orientationErrorDeg"] = firmware_errors["maxRpyErrorDeg"]
            if not firmware_errors["withinTolerance"]:
                result["rejectionReasons"].append("firmware_fk_roundtrip_residual")
            elif (
                firmware_errors["maxPositionErrorMm"] > COORD_TARGET_TOLERANCE_MM
                or firmware_errors["maxRpyErrorDeg"] > COORD_RPY_TOLERANCE_DEG
            ):
                result.setdefault("accuracyWarnings", []).append(
                    "firmware_fk_roundtrip_above_precision_target"
                )
        else:
            # Offline/fake-driver fallback uses the independent host FK as the
            # displayed round-trip residual. The hard host gates above still
            # apply in both connected and disconnected previews.
            result["positionErrorMm"] = result["hostPositionErrorMm"]
            result["orientationErrorDeg"] = result["hostOrientationErrorDeg"]
        result["ok"] = not result["rejectionReasons"]
        return result

    @classmethod
    def _minimum_inward_shift_mm(
        cls,
        steps: List[Dict[str, Any]],
        start_angles: List[float],
        desired_jaw_yaw: float,
        tool_id: str = "adaptive_gripper",
        correction_local_m: Optional[List[float]] = None,
        suction_contact_distance_m: float = 0.072,
        tilt_candidates: Optional[List[float]] = None,
    ) -> Optional[float]:
        """Estimate the smallest safe radial relocation using host IK only.

        This is guidance, never a rewritten motion target. All states in the
        group must solve with one fixed orientation and continuous joints.
        """
        tcp_steps = [step for step in steps if isinstance(step.get("targetTcpPoseM"), dict)]
        if not tcp_steps:
            return None
        anchor = tcp_steps[0]["targetTcpPoseM"]
        anchor_x = float(anchor.get("x", 0.0))
        anchor_y = float(anchor.get("y", 0.0))
        radius = math.hypot(anchor_x, anchor_y)
        if radius < 1e-6:
            return None
        unit_x, unit_y = anchor_x / radius, anchor_y / radius
        radial_yaw = math.atan2(anchor_y, anchor_x)
        def solves(shift_mm: float, tilt: float) -> bool:
            shift_m = shift_mm / 1000.0
            tcp_rotation = grasp_rotation(desired_jaw_yaw, tilt, radial_yaw)
            current = list(start_angles)
            for step in tcp_steps:
                target = step["targetTcpPoseM"]
                target_tcp = (
                    float(target["x"]) - unit_x * shift_m,
                    float(target["y"]) - unit_y * shift_m,
                    float(target["z"]),
                )
                model_tcp = tuple(
                    target_tcp[index] - FIRMWARE_BASE_TRANSLATION_M[index]
                    for index in range(3)
                )
                solved = solve_pose(
                    model_tcp, tcp_rotation, current, exhaustive=False,
                    tool_id=tool_id, correction_local_m=correction_local_m,
                    suction_contact_distance_m=suction_contact_distance_m,
                )
                if solved is None or not cls._joint_solution_diagnostics(solved, current).get("ok"):
                    return False
                current = list(solved)
            return True

        # Exponentially bracket the first reachable inward relocation. This is
        # logarithmic like a max/binary search, but also handles inner
        # singularities where a pose reachable after 5 mm is not reachable
        # after moving the full 120 mm inward.
        low = 0.0
        high: Optional[float] = None
        viable_tilt: Optional[float] = None
        tilt_order = list(dict.fromkeys(
            float(value) for value in (tilt_candidates or list(IK_ORIENTATION_TILT_OFFSETS_DEG))
        ))
        probe = float(INWARD_REACH_SEARCH_STEP_MM)
        while probe <= INWARD_REACH_SEARCH_MAX_MM:
            viable_tilt = next(
                (tilt for tilt in tilt_order if solves(probe, tilt)),
                None,
            )
            if viable_tilt is not None:
                high = probe
                break
            low = probe
            probe = min(float(INWARD_REACH_SEARCH_MAX_MM), probe * 2.0)
            if probe == low:
                break
        if high is None or viable_tilt is None:
            return None
        while high - low > INWARD_REACH_SEARCH_STEP_MM:
            midpoint = round(
                ((low + high) / 2.0) / INWARD_REACH_SEARCH_STEP_MM
            ) * INWARD_REACH_SEARCH_STEP_MM
            if midpoint <= low or midpoint >= high:
                break
            if solves(midpoint, viable_tilt):
                high = midpoint
            else:
                low = midpoint
        return float(high)

    def _preview_fixed_taught_group(
        self, robot: MyCobotDriver, steps: List[Dict[str, Any]], start_angles: List[float]
    ) -> Dict[str, Any]:
        """Validate captured waypoint poses without rewriting their orientation."""
        current = [float(value) for value in start_angles[:6]]
        states: List[Dict[str, Any]] = []
        previous_coords: Optional[List[float]] = None
        for step in steps:
            state_id = str(step.get("stateId") or step.get("name") or "unknown")
            coords = [float(value) for value in (step.get("coordsMm") or [])]
            if len(coords) != 6:
                return {"ok": False, "states": states, "error": f"{state_id} has incomplete taught coordinates."}
            bounds = validate_coordinate_bounds(coords, state_id, allow_missing_rpy=False)
            if bounds:
                state = {"stateId": state_id, "targetCoords": coords, "ok": False, "rejectionReasons": ["coordinate_bounds"]}
                states.append(state)
                return {"ok": False, "states": states, "error": bounds[0].get("message") or "Taught point is outside coordinate bounds."}
            tool_id = str(step.get("activeTool") or self.end_effector)
            profile = step.get("toolProfile") or self.tool_profile or {}
            correction = profile.get("tcpCorrectionLocalM") or {}
            correction_local_m = [float(correction.get(axis, 0.0)) for axis in ("x", "y", "z")]
            suction_distance = float((profile.get("geometry") or {}).get("flangeToContactM", 0.072))
            waypoint_coords = [coords]
            if int(step.get("coordMode", 0)) == 1 and previous_coords is not None:
                distance_mm = math.dist(previous_coords[:3], coords[:3])
                count = max(1, int(math.ceil(distance_mm / 30.0)))
                waypoint_coords = [
                    [
                        previous_coords[axis] + (coords[axis] - previous_coords[axis]) * index / count
                        for axis in range(3)
                    ] + [
                        previous_coords[axis] + (
                            ((coords[axis] - previous_coords[axis] + 180.0) % 360.0) - 180.0
                        ) * index / count
                        for axis in range(3, 6)
                    ]
                    for index in range(1, count + 1)
                ]
            preferred = step.get("preferredJointSeedDeg")
            waypoint_records = []
            diagnostics: Dict[str, Any] = {"ok": False, "rejectionReasons": ["no_waypoint_solution"]}
            for waypoint_index, waypoint in enumerate(waypoint_coords, 1):
                seeds = [list(current)]
                if waypoint_index == len(waypoint_coords) and isinstance(preferred, list) and len(preferred) >= 6:
                    candidate_seed = [float(value) for value in preferred[:6]]
                    if any(abs(candidate_seed[i] - current[i]) > 0.01 for i in range(6)):
                        seeds.append(candidate_seed)
                solutions = []
                failures = []
                for seed in seeds:
                    try:
                        solved = robot.solve_inv_kinematics(waypoint, seed)
                        firmware_fk = robot.angles_to_coords(solved) if hasattr(robot, "angles_to_coords") else None
                        candidate = self._validate_firmware_ik(
                            waypoint, solved, current, firmware_fk, tool_id,
                            correction_local_m, suction_distance,
                        )
                        if candidate.get("ok"):
                            solutions.append(candidate)
                        else:
                            failures.append(candidate)
                    except Exception as exc:
                        failures.append({"ok": False, "rejectionReasons": [f"firmware_ik_error: {exc}"]})
                if not solutions:
                    diagnostics = max(
                        failures,
                        key=lambda item: int(bool(item.get("angles"))),
                        default={"ok": False, "rejectionReasons": ["no_waypoint_solution"]},
                    )
                    break
                diagnostics = min(
                    solutions,
                    key=lambda item: (
                        float(item.get("maxJointStepDeg") or 999.0),
                        float(item.get("positionErrorMm") or 999.0),
                        float(item.get("orientationErrorDeg") or 999.0),
                    ),
                )
                current = [float(value) for value in diagnostics["angles"]]
                waypoint_records.append({
                    "index": waypoint_index,
                    "coords": [round(float(value), 3) for value in waypoint],
                    "maxJointStepDeg": diagnostics.get("maxJointStepDeg"),
                    "ok": True,
                })
            diagnostics["subdivisionWaypointCount"] = len(waypoint_coords)
            diagnostics["subdivision"] = waypoint_records
            state = {
                "stateId": state_id,
                "targetCoords": [round(value, 6) for value in coords],
                **diagnostics,
            }
            states.append(state)
            if not diagnostics.get("ok"):
                return {
                    "ok": False,
                    "states": states,
                    "error": f"Taught point {step.get('pointLabel') or step.get('pointId')} failed independent IK validation.",
                }
            previous_coords = coords
            step["previewAngles"] = [round(value, 2) for value in current]
            step["selectedOrientation"] = {
                "rpyDeg": [round(value, 3) for value in coords[3:6]],
                "policy": "fixed_taught_pose",
            }
            step["ikValidation"] = {
                key: value for key, value in diagnostics.items()
                if key not in ("angles", "targetCoords", "stateId", "ok")
            }
        return {"ok": True, "states": states, "endAngles": current, "error": None}

    def _preview_coordinate_group(
        self, robot: MyCobotDriver, steps: List[Dict[str, Any]], start_angles: List[float],
        _orientation_filter: Optional[set] = None,
        _host_exhaustive: bool = False,
        _allow_exhaustive_fallback: bool = True,
        _ik_cache: Optional[Dict[Any, Dict[str, Any]]] = None,
        _stats: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        if steps and all(step.get("orientationPolicy") == "fixed_taught_pose" for step in steps):
            return self._preview_fixed_taught_group(robot, steps, start_angles)
        tool_id = str(steps[0].get("activeTool") or self.end_effector)
        step_profile = steps[0].get("toolProfile") or self.tool_profile or {}
        raw_correction = step_profile.get("tcpCorrectionLocalM") or {}
        correction_local_m = [float(raw_correction.get(axis, 0.0)) for axis in ("x", "y", "z")]
        suction_contact_distance_m = float(
            (step_profile.get("geometry") or {}).get("flangeToContactM", 0.072)
        )
        base = list(steps[0].get("coordsMm") or [])
        if len(base) != 6 or any(value is None for value in base):
            return {"ok": False, "states": [], "error": "Coordinate group has no complete captured RPY."}
        configured_base_rpy = steps[0].get("baseToolRpyDeg")
        if isinstance(configured_base_rpy, list) and len(configured_base_rpy) >= 3:
            base[3:6] = [float(value) for value in configured_base_rpy[:3]]
        base_axes = tool_axis_diagnostics(rotation_from_rpy_deg(base[3:6]), tool_id)
        configured_jaw_yaw = steps[0].get("desiredJawYawDeg")
        free_carry_yaw = configured_jaw_yaw is None
        suction_j6_locked = tool_id == "suction_gripper" and free_carry_yaw
        locked_j6_deg = float(start_angles[5]) if suction_j6_locked else None
        desired_jaw_yaw = configured_jaw_yaw
        if desired_jaw_yaw is None:
            desired_jaw_yaw = base_axes["jawYawDeg"]
        if float(base_axes["approachTiltDeg"]) > MAX_TOP_DOWN_TILT_DEG:
            return {
                "ok": False,
                "states": [],
                "error": f"Configured tool orientation is sideways ({float(base_axes['approachTiltDeg']):.2f} deg from vertical).",
            }
        candidates = []
        rejected: List[Dict[str, Any]] = []
        orientation_candidates = []
        ik_cache = _ik_cache if _ik_cache is not None else {}
        stats = _stats if _stats is not None else {
            "orientationCandidates": 0,
            "fastHostSolves": 0,
            "exhaustiveHostSolves": 0,
            "hostCacheHits": 0,
            "firmwareIkCalls": 0,
            "exhaustiveFallbackCandidates": 0,
        }
        yaw_offsets = (
            (0.0,)
            if suction_j6_locked
            else (IK_FREE_CARRY_YAW_OFFSETS_DEG if free_carry_yaw else IK_ORIENTATION_YAW_OFFSETS_DEG)
        )
        tcp_targets = [
            step.get("targetTcpPoseM") or {}
            for step in steps
            if all(axis in (step.get("targetTcpPoseM") or {}) for axis in ("x", "y"))
        ]
        # A fixed tilt must lean toward the limiting radial target.  Carry
        # groups often start on the near side of the base and end at a far bin;
        # using the first waypoint's radial axis tilts the tool the wrong way
        # at the actual reach boundary.
        radial_anchor = max(
            tcp_targets,
            key=lambda target: math.hypot(float(target["x"]), float(target["y"])),
            default=steps[0].get("targetTcpPoseM") or {},
        )
        radial_yaw = math.atan2(
            float(radial_anchor.get("y", base[1] / 1000.0)),
            float(radial_anchor.get("x", base[0] / 1000.0)),
        )
        # Evaluate every equivalent yaw vertically before allowing any tilt;
        # then increase tilt deterministically. The first complete valid path
        # wins, so vertical is always preferred and preview remains quick.
        for tilt in IK_ORIENTATION_TILT_OFFSETS_DEG:
            for yaw_offset in yaw_offsets:
                tcp_rotation = grasp_rotation(
                    float(desired_jaw_yaw) + float(yaw_offset), float(tilt), radial_yaw
                )
                _, flange_rotation = flange_from_tcp(
                    (0.0, 0.0, 0.0), tcp_rotation, tool_id,
                    correction_local_m, suction_contact_distance_m,
                )
                candidate_rpy = [float(value) for value in rpy_deg_from_rotation(flange_rotation)]
                orientation_candidates.append(
                    (float(tilt), float(yaw_offset), candidate_rpy, tcp_rotation)
                )
        preferred_orientation = steps[0].get("preferredOrientation") or {}
        preferred_key = (
            float(preferred_orientation.get("tiltOffsetDeg", float("nan"))),
            float(preferred_orientation.get("yawOffsetDeg", float("nan"))),
        )
        orientation_candidates.sort(key=lambda item: (
            0 if (item[0], item[1]) == preferred_key else 1,
            abs(item[0]), abs(item[1]),
        ))
        if _orientation_filter is not None:
            orientation_candidates = [
                item for item in orientation_candidates
                if (item[0], item[1]) in _orientation_filter
            ]
        stats["orientationCandidates"] += len(orientation_candidates)

        def host_solution_for(
            waypoint: List[float], seed: List[float]
        ) -> Tuple[Optional[List[float]], Dict[str, Any]]:
            key = (
                tuple(round(float(value), 3) for value in waypoint),
                tuple(round(float(value), 2) for value in seed),
                tool_id,
                tuple(round(value, 6) for value in correction_local_m),
                round(suction_contact_distance_m, 6),
                bool(_host_exhaustive),
            )
            cached = ik_cache.get(key)
            if cached is not None:
                stats["hostCacheHits"] += 1
                return deepcopy(cached.get("angles")), deepcopy(cached)
            target_flange_position = tuple(float(value) / 1000.0 for value in waypoint[:3])
            target_flange_rotation = rotation_from_rpy_deg(waypoint[3:6])
            target_tcp_position, target_tcp_rotation = tcp_from_flange(
                target_flange_position, target_flange_rotation, tool_id,
                correction_local_m, suction_contact_distance_m,
            )
            model_tcp_position = tuple(
                target_tcp_position[index] - FIRMWARE_BASE_TRANSLATION_M[index]
                for index in range(3)
            )
            diagnostics: Dict[str, Any] = {}
            solution = solve_pose(
                model_tcp_position, target_tcp_rotation, seed,
                exhaustive=_host_exhaustive, tool_id=tool_id,
                correction_local_m=correction_local_m,
                suction_contact_distance_m=suction_contact_distance_m,
                diagnostics=diagnostics,
            )
            stats["exhaustiveHostSolves" if _host_exhaustive else "fastHostSolves"] += 1
            diagnostics["angles"] = list(solution) if solution is not None else None
            ik_cache[key] = deepcopy(diagnostics)
            return solution, diagnostics
        for tilt, yaw, rpy, candidate_tcp_rotation in orientation_candidates:
                current = list(start_angles)
                states = []
                total_error = total_travel = 0.0
                previous_coords = None
                for step in steps:
                    requested_tcp = step.get("targetTcpPoseM") or {}
                    step_tcp_rotation = candidate_tcp_rotation
                    step_rpy = list(rpy)
                    if all(axis in requested_tcp for axis in ("x", "y", "z")):
                        requested_tcp_position = (
                            float(requested_tcp["x"]),
                            float(requested_tcp["y"]),
                            float(requested_tcp["z"]),
                        )
                        if suction_j6_locked and locked_j6_deg is not None:
                            current_flange_position, current_flange_rotation = firmware_flange_kinematics(current)
                            current_tcp_position, _ = tcp_from_flange(
                                current_flange_position, current_flange_rotation, tool_id,
                                correction_local_m, suction_contact_distance_m,
                            )
                            current_axes = tool_axis_diagnostics(current_flange_rotation, tool_id)
                            current_bearing = math.atan2(current_tcp_position[1], current_tcp_position[0])
                            target_bearing = math.atan2(requested_tcp_position[1], requested_tcp_position[0])
                            bearing_change_deg = math.degrees(
                                ((target_bearing - current_bearing + math.pi) % (2.0 * math.pi)) - math.pi
                            )
                            step_jaw_yaw = float(current_axes["jawYawDeg"]) + bearing_change_deg
                            step_tcp_rotation = grasp_rotation(
                                step_jaw_yaw, float(tilt), radial_yaw
                            )
                            _, initial_probe_rotation = flange_from_tcp(
                                requested_tcp_position, step_tcp_rotation, tool_id,
                                correction_local_m, suction_contact_distance_m,
                            )
                            step_rpy = [
                                float(value) for value in rpy_deg_from_rotation(initial_probe_rotation)
                            ]
                            # J6 is rotation around the round cup's own axis.
                            # Adjust only the otherwise-free suction yaw until
                            # firmware IK returns the locked starting J6.
                            # The round suction cup is yaw-symmetric. One
                            # correction after the initial firmware result is
                            # sufficient; the final lock tolerance remains the
                            # authoritative acceptance gate.
                            for _ in range(2):
                                step_tcp_rotation = grasp_rotation(step_jaw_yaw, float(tilt), radial_yaw)
                                probe_flange_position, probe_flange_rotation = flange_from_tcp(
                                    requested_tcp_position, step_tcp_rotation, tool_id,
                                    correction_local_m, suction_contact_distance_m,
                                )
                                probe_rpy = [float(value) for value in rpy_deg_from_rotation(probe_flange_rotation)]
                                probe_coords = [float(value) * 1000.0 for value in probe_flange_position] + probe_rpy
                                try:
                                    # Screen the yaw-correction pose with the
                                    # independent host model before involving
                                    # connected firmware. The cache lets the
                                    # normal waypoint check reuse this solve.
                                    probe_host, _ = host_solution_for(probe_coords, current)
                                    if probe_host is None:
                                        break
                                    if isinstance(robot, HostKinematicsPreviewRobot):
                                        probe_angles = list(probe_host)
                                    else:
                                        stats["firmwareIkCalls"] += 1
                                        probe_angles = robot.solve_inv_kinematics(probe_coords, current)
                                    if not isinstance(probe_angles, (list, tuple)) or len(probe_angles) < 6:
                                        break
                                    signed_lock_error = (
                                        (float(probe_angles[5]) - locked_j6_deg + 180.0) % 360.0
                                    ) - 180.0
                                    step_rpy = probe_rpy
                                    if abs(signed_lock_error) <= 0.05:
                                        break
                                    # For this tool transform, firmware J6
                                    # counter-rotates against TCP yaw.
                                    step_jaw_yaw += signed_lock_error
                                except Exception:
                                    break
                        candidate_flange_position, _ = flange_from_tcp(
                            requested_tcp_position,
                            step_tcp_rotation,
                            tool_id, correction_local_m, suction_contact_distance_m,
                        )
                        coords = [float(value) * 1000.0 for value in candidate_flange_position] + step_rpy
                    else:
                        coords = [float(value) for value in step["coordsMm"][:3]] + step_rpy
                    bounds = validate_coordinate_bounds(
                        coords,
                        str(step.get("stateId") or "unknown"),
                        allow_missing_rpy=False,
                        xy_margin_mm=generated_coordinate_xy_margin_mm(step),
                    )
                    if bounds:
                        states.append({"stateId": step.get("stateId"), "targetCoords": coords, "rejectionReasons": ["coordinate_bounds"]})
                        break
                    waypoint_coords = [coords]
                    if int(step.get("coordMode", 0)) == 1 and previous_coords is not None:
                        distance_mm = math.dist(previous_coords[:3], coords[:3])
                        count = max(1, int(math.ceil(distance_mm / 30.0)))
                        waypoint_coords = [
                            [
                                previous_coords[axis] + (coords[axis] - previous_coords[axis]) * index / count
                                for axis in range(3)
                            ] + step_rpy
                            for index in range(1, count + 1)
                        ]
                    waypoint_records = []
                    diagnostics: Dict[str, Any] = {"ok": False, "rejectionReasons": ["no_waypoint_solution"]}
                    try:
                        for waypoint_index, waypoint in enumerate(waypoint_coords, 1):
                            host_solution, host_diagnostics = host_solution_for(waypoint, current)
                            preferred_seed = step.get("preferredJointSeedDeg")
                            if (
                                host_solution is None
                                and isinstance(preferred_seed, list)
                                and len(preferred_seed) >= 6
                            ):
                                host_solution, host_diagnostics = host_solution_for(
                                    waypoint, [float(value) for value in preferred_seed[:6]]
                                )
                            if host_solution is None:
                                diagnostics = {
                                    "ok": False,
                                    "rejectionReasons": [
                                        "host_ik_unreachable" if _host_exhaustive
                                        else "host_ik_fast_screen_failed"
                                    ],
                                    "hostIkReachable": False,
                                    "hostBestPositionErrorMm": (
                                        round(float(host_diagnostics["bestPositionErrorM"]) * 1000.0, 3)
                                        if host_diagnostics.get("bestPositionErrorM") is not None else None
                                    ),
                                    "hostBestOrientationErrorDeg": (
                                        round(math.degrees(float(host_diagnostics["bestOrientationErrorRad"])), 3)
                                        if host_diagnostics.get("bestOrientationErrorRad") is not None else None
                                    ),
                                    "hostSeedsAttempted": host_diagnostics.get("seedsAttempted"),
                                }
                                waypoint_records.append({
                                    "index": waypoint_index,
                                    "coords": [round(value, 3) for value in waypoint],
                                    "ok": False,
                                })
                                break
                            if isinstance(robot, HostKinematicsPreviewRobot):
                                solved = list(host_solution)
                            else:
                                stats["firmwareIkCalls"] += 1
                                solved = robot.solve_inv_kinematics(waypoint, current)
                            firmware_fk = robot.angles_to_coords(solved) if hasattr(robot, "angles_to_coords") else None
                            diagnostics = self._validate_firmware_ik(
                                waypoint, solved, current, firmware_fk, tool_id,
                                correction_local_m, suction_contact_distance_m,
                                host_solution_override=host_solution,
                            )
                            target_flange_position = tuple(float(value) / 1000.0 for value in waypoint[:3])
                            target_flange_rotation = rotation_from_rpy_deg(waypoint[3:6])
                            target_tcp_position, _ = tcp_from_flange(
                                target_flange_position, target_flange_rotation, tool_id,
                                correction_local_m, suction_contact_distance_m,
                            )
                            actual_flange_position, actual_flange_rotation = firmware_flange_kinematics(diagnostics.get("angles") or solved)
                            actual_tcp_position, _ = tcp_from_flange(
                                actual_flange_position, actual_flange_rotation, tool_id,
                                correction_local_m, suction_contact_distance_m,
                            )
                            jaw_center_error = math.dist(actual_tcp_position, target_tcp_position) * 1000.0
                            axes = tool_axis_diagnostics(actual_flange_rotation, tool_id)
                            diagnostics["jawCenterErrorMm"] = round(jaw_center_error, 3)
                            diagnostics["toolApproachTiltDeg"] = round(float(axes["approachTiltDeg"]), 3)
                            diagnostics["jawYawDeg"] = round(float(axes["jawYawDeg"]), 3)
                            diagnostics["toolApproachAxis"] = [round(float(value), 6) for value in axes["approachAxis"]]
                            diagnostics["jawAxis"] = [round(float(value), 6) for value in axes["jawAxis"]]
                            if suction_j6_locked and locked_j6_deg is not None:
                                actual_j6 = float((diagnostics.get("angles") or solved)[5])
                                j6_lock_error = abs(
                                    ((actual_j6 - locked_j6_deg + 180.0) % 360.0) - 180.0
                                )
                                diagnostics["suctionJ6LockDeg"] = round(locked_j6_deg, 3)
                                diagnostics["suctionJ6LockErrorDeg"] = round(j6_lock_error, 3)
                                if j6_lock_error > SUCTION_J6_LOCK_TOLERANCE_DEG:
                                    diagnostics["rejectionReasons"].append("suction_j6_lock_violation")
                            if jaw_center_error > MAX_IK_JAW_CENTER_ERROR_MM:
                                diagnostics["rejectionReasons"].append("jaw_center_residual")
                            elif jaw_center_error > COORD_TARGET_TOLERANCE_MM:
                                diagnostics.setdefault("accuracyWarnings", []).append(
                                    "jaw_center_above_precision_target"
                                )
                            if float(axes["approachTiltDeg"]) > MAX_TOP_DOWN_TILT_DEG:
                                diagnostics["rejectionReasons"].append("tool_not_top_down")
                            diagnostics["ok"] = not diagnostics["rejectionReasons"]
                            waypoint_records.append({
                                "index": waypoint_index,
                                "coords": [round(value, 3) for value in waypoint],
                                "maxJointStepDeg": diagnostics.get("maxJointStepDeg"),
                                "jawCenterErrorMm": diagnostics.get("jawCenterErrorMm"),
                                "ok": diagnostics.get("ok"),
                            })
                            total_travel += float(diagnostics.get("maxJointStepDeg") or 0.0)
                            if not diagnostics.get("ok"):
                                break
                            current = diagnostics["angles"]
                    except Exception as exc:
                        diagnostics = {"ok": False, "rejectionReasons": [f"firmware_ik_error: {exc}"]}
                    target_flange_position = tuple(float(value) / 1000.0 for value in coords[:3])
                    target_flange_rotation = rotation_from_rpy_deg(coords[3:6])
                    planned_tcp_position, _ = tcp_from_flange(
                        target_flange_position, target_flange_rotation, tool_id,
                        correction_local_m, suction_contact_distance_m,
                    )
                    requested_tcp = step.get("targetTcpPoseM") or {}
                    planned_error = math.dist(
                        planned_tcp_position,
                        (
                            float(requested_tcp.get("x", planned_tcp_position[0])),
                            float(requested_tcp.get("y", planned_tcp_position[1])),
                            float(requested_tcp.get("z", planned_tcp_position[2])),
                        ),
                    ) * 1000.0
                    diagnostics["plannedJawCenterErrorMm"] = round(planned_error, 3)
                    diagnostics["subdivisionWaypointCount"] = len(waypoint_coords)
                    diagnostics["subdivision"] = waypoint_records
                    if planned_error > MAX_PLANNED_JAW_CENTER_ERROR_MM:
                        diagnostics.setdefault("rejectionReasons", []).append("planned_flange_tcp_mismatch")
                        diagnostics["ok"] = False
                    state = {"stateId": step.get("stateId"), "targetCoords": [round(v, 3) for v in coords], **diagnostics}
                    states.append(state)
                    if not diagnostics.get("ok"):
                        break
                    total_error += diagnostics["positionErrorMm"] + diagnostics["orientationErrorDeg"]
                    previous_coords = coords
                if len(states) == len(steps) and all(state.get("ok") for state in states):
                    j6_travel = sum(float(state.get("joint6StepDeg", 0.0)) for state in states)
                    candidates.append((total_error + total_travel * 0.02 + j6_travel * 0.08 + abs(tilt) * 0.1, tilt, yaw, rpy, states, current))
                    break
                else:
                    failed_state = next((state for state in states if not state.get("ok")), {})
                    rejected.append({
                        "tiltDeg": tilt,
                        "yawOffsetDeg": yaw,
                        "states": states,
                        "residualScore": (
                            float(failed_state.get("hostBestPositionErrorMm") or 1e6)
                            + float(failed_state.get("hostBestOrientationErrorDeg") or 1e6)
                        ),
                    })
        if not candidates:
            if not _host_exhaustive and _allow_exhaustive_fallback and rejected:
                ranked = sorted(
                    rejected,
                    key=lambda item: (
                        -sum(1 for state in item.get("states") or [] if state.get("ok")),
                        float(item.get("residualScore") or 1e9),
                        abs(float(item.get("tiltDeg") or 0.0)),
                    ),
                )[:2]
                fallback_filter = {
                    (float(item["tiltDeg"]), float(item["yawOffsetDeg"]))
                    for item in ranked
                }
                stats["exhaustiveFallbackCandidates"] += len(fallback_filter)
                return self._preview_coordinate_group(
                    robot,
                    steps,
                    start_angles,
                    _orientation_filter=fallback_filter,
                    _host_exhaustive=True,
                    _allow_exhaustive_fallback=False,
                    _ik_cache=ik_cache,
                    _stats=stats,
                )
            best_rejection = max(rejected, key=lambda item: len(item.get("states") or []), default=None)
            best_states = best_rejection.get("states") if best_rejection else []
            first_failed = next((state for state in best_states if not state.get("ok")), None)
            first_reasons = list((first_failed or {}).get("rejectionReasons") or [])
            inward_shift = self._minimum_inward_shift_mm(
                steps, start_angles, float(desired_jaw_yaw), tool_id,
                correction_local_m, suction_contact_distance_m,
                tilt_candidates=[
                    float(item.get("tiltDeg") or 0.0)
                    for item in sorted(
                        rejected,
                        key=lambda item: float(item.get("residualScore") or 1e9),
                    )[:2]
                ],
            )
            guidance = None
            if "joint_discontinuity" in first_reasons:
                guidance = (
                    "The target pose is reachable, but this transfer still needs an additional "
                    "joint-continuity waypoint. The destination itself does not need to move."
                )
            elif inward_shift is not None:
                guidance = (
                    f"Move the object at least {inward_shift:.0f} mm toward the robot base "
                    "and plan again."
                )
            if "joint_discontinuity" in first_reasons:
                error = "A reachable coordinate target was rejected because the transfer path was not sufficiently subdivided."
            else:
                error = "No vertical or <=10 deg tilted orientation passed independent full-path IK validation."
            if guidance:
                error += f" {guidance}"
            return {
                "ok": False,
                "states": best_states,
                "error": error,
                "suggestedInwardShiftMm": inward_shift,
                "correctiveGuidance": guidance,
                "rejectedCandidates": rejected,
                "planningDiagnostics": deepcopy(stats),
            }
        _, tilt, yaw, rpy, states, end_angles = min(candidates, key=lambda item: item[0])
        by_id = {state["stateId"]: state for state in states}
        for step in steps:
            state = by_id.get(step.get("stateId"), {})
            selected_coords = list(state.get("targetCoords") or [])
            selected_rpy = selected_coords[3:6] if len(selected_coords) == 6 else list(rpy)
            selected_axes = tool_axis_diagnostics(rotation_from_rpy_deg(selected_rpy), tool_id)
            if len(selected_coords) == 6:
                # Orientation alternatives change the lateral flange offset of
                # this asymmetrically mounted gripper. Persist the complete
                # selected flange pose, not only its RPY, so execution sends
                # the exact pose that passed jaw-center validation.
                step["coordsMm"] = [round(float(value), 3) for value in selected_coords]
                step["targetFlangePoseM"] = {
                    "x": round(float(selected_coords[0]) / 1000.0, 6),
                    "y": round(float(selected_coords[1]) / 1000.0, 6),
                    "z": round(float(selected_coords[2]) / 1000.0, 6),
                }
            step["previewAngles"] = [round(value, 2) for value in state.get("angles", [])]
            step["selectedOrientation"] = {
                "rpyDeg": [round(value, 3) for value in selected_rpy],
                "tiltOffsetDeg": tilt,
                "yawOffsetDeg": yaw,
                "jawYawDeg": round(float(selected_axes["jawYawDeg"]), 3),
                "toolApproachTiltDeg": round(float(selected_axes["approachTiltDeg"]), 3),
            }
            if suction_j6_locked and locked_j6_deg is not None:
                step["suctionJ6LockDeg"] = round(locked_j6_deg, 3)
            step["ikValidation"] = {key: value for key, value in state.items() if key not in ("angles", "targetCoords", "stateId", "ok")}
        return {
            "ok": True,
            "states": states,
            "endAngles": end_angles,
            "error": None,
            "planningDiagnostics": deepcopy(stats),
        }

    def send_angles(self, angles: Any, speed: Any) -> Dict[str, Any]:
        try:
            values = [float(value) for value in angles]
            speed_value = int(speed)
        except (TypeError, ValueError):
            return {"ok": False, "error": "Joint command requires six finite angles and a numeric speed.", **self.status()}
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            return {"ok": False, "error": "Joint command requires exactly six finite angles.", **self.status()}
        if speed_value < 1 or speed_value > 100:
            return {"ok": False, "error": "Joint speed must be between 1 and 100.", **self.status()}
        try:
            for joint, value in enumerate(values, 1):
                validate_joint_angle(joint, value)
        except ValueError as exc:
            return {"ok": False, "error": str(exc), **self.status()}
        with self.lock:
            if self.executing:
                return {"ok": False, "error": "A physical plan is running; press Stop first.", **self.status_locked()}
            if self.jog_session:
                return {"ok": False, "error": "Stop joint jogging before sending a combined angle target.", **self.status_locked()}
            try:
                self.get_robot_locked().send_angles(values, speed_value)
                self.last_error = None
                return {"ok": True, "sentAngles": values, "speed": speed_value, **self.status_locked()}
            except SerialException as exc:
                # Fatal link error: drop the port so the next call reconnects.
                self.close_locked()
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}
            except Exception as exc:
                # Transient read miss: keep the port; the next call retries.
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}

    def command(self, name: str) -> Dict[str, Any]:
        allowed = {
            "focus-all", "power-on", "release", "stop",
            "suction-pump-on", "suction-pump-off",
            "suction-valve-open", "suction-valve-close",
            "gripper-open", "gripper-close", "gripper-auto", "gripper-release",
            "suction-on", "suction-off", "home", "zero",
        }
        if name not in allowed:
            return {"ok": False, "error": f"Unknown command: {name}", **self.status()}
        if name in ("stop", "release"):
            # Signal a running plan to bail out before competing for the serial lock.
            self.abort_event.set()
        with self.lock:
            if self.executing and name not in ("stop", "release"):
                return {
                    "ok": False,
                    "error": "A physical plan is running; only stop or release are accepted.",
                    **self.status_locked(),
                }
            try:
                robot = self.get_robot_locked()
                if name in ("stop", "release"):
                    self._stop_jog_locked("operator_stop")
                if name == "focus-all":
                    robot.focus_all_servos()
                elif name == "power-on":
                    robot.power_on()
                elif name == "release":
                    robot.release_all_servos()
                elif name == "stop":
                    robot.stop()
                elif name == "suction-pump-on":
                    suction = robot.set_suction_output(5, 0)
                elif name == "suction-pump-off":
                    suction = robot.set_suction_output(5, 1)
                elif name == "suction-valve-open":
                    suction = robot.set_suction_output(2, 0)
                elif name == "suction-valve-close":
                    suction = robot.set_suction_output(2, 1)
                elif name in ("gripper-open", "suction-off"):
                    if self.end_effector == "suction_gripper":
                        suction = robot.suction_off()
                    else:
                        suction = None
                        robot.open_gripper()
                elif name in ("gripper-close", "suction-on"):
                    if self.end_effector == "suction_gripper":
                        suction = robot.suction_on()
                    else:
                        suction = None
                        robot.close_gripper()
                elif name == "gripper-auto":
                    if self.end_effector == "suction_gripper":
                        suction = robot.suction_on()
                    else:
                        suction = None
                        robot.auto_grip(speed=35)
                elif name == "gripper-release":
                    if self.end_effector == "suction_gripper":
                        suction = robot.suction_off()
                    else:
                        suction = None
                        robot.release_gripper()
                elif name in ("home", "zero"):
                    suction = None
                    robot.send_angles(HOME_ANGLES, 15)
                self.last_error = None
                result = {"ok": True, "command": name, **self.status_locked()}
                if name.startswith("suction-") or (name.startswith("gripper-") and self.end_effector == "suction_gripper"):
                    result["suction"] = {
                        "enabled": name in ("gripper-close", "gripper-auto", "suction-on"),
                        **(suction or {}),
                    }
                return result
            except SerialException as exc:
                # Fatal link error: drop the port so the next call reconnects.
                self.close_locked()
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}
            except Exception as exc:
                # Transient read miss: keep the port; the next call retries.
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}

    @staticmethod
    def angle_error_deg(target: List[float], actual: List[float]) -> float:
        errors = [abs(((float(t) - float(a) + 180.0) % 360.0) - 180.0) for t, a in zip(target, actual)]
        return max(errors) if errors else 999.0

    def read_angles_locked(self, robot: MyCobotDriver) -> List[float]:
        angles = robot.get_angles()
        self.last_angles = angles
        self.last_error = None
        self.last_read_at = time.time()
        return angles

    def read_coords_locked(self, robot: MyCobotDriver) -> List[float]:
        coords = robot.get_coords()
        self.last_coords = coords
        self.last_error = None
        self.last_read_at = time.time()
        return coords

    @staticmethod
    def coords_error(
        target: List[float], actual: Optional[List[float]],
        position_tolerance_mm: float = COORD_TARGET_TOLERANCE_MM,
        rpy_tolerance_deg: float = COORD_RPY_TOLERANCE_DEG,
    ) -> Dict[str, Any]:
        if actual is None or len(actual) < 6 or len(target) < 6:
            return {"maxPositionErrorMm": None, "maxRpyErrorDeg": None, "withinTolerance": False}
        position_errors = [abs(float(target[i]) - float(actual[i])) for i in range(3)]
        rpy_errors = [
            abs(((float(target[i]) - float(actual[i]) + 180.0) % 360.0) - 180.0)
            for i in range(3, 6)
        ]
        max_pos = max(position_errors)
        max_rpy = max(rpy_errors)
        return {
            "maxPositionErrorMm": round(max_pos, 2),
            "maxRpyErrorDeg": round(max_rpy, 2),
            "withinTolerance": bool(
                max_pos <= position_tolerance_mm
                and max_rpy <= rpy_tolerance_deg
            ),
            "positionToleranceMm": float(position_tolerance_mm),
            "rpyToleranceDeg": float(rpy_tolerance_deg),
        }

    def resolve_step_coords(
        self,
        robot: MyCobotDriver,
        step: Dict[str, Any],
        runtime_rpy: Optional[List[float]],
    ) -> tuple[List[float], Optional[List[float]], str]:
        raw = step.get("coordsMm")
        if not isinstance(raw, list) or len(raw) != 6:
            raise ValueError(f"State {step.get('stateId') or step.get('name')} needs coordsMm [x,y,z,rx,ry,rz].")
        coords: List[Optional[float]] = []
        for index, value in enumerate(raw):
            if value is None and index >= 3:
                coords.append(None)
            else:
                coords.append(float(value))
        rpy_source = str(step.get("toolRpySource") or "captured")
        if any(value is None for value in coords[3:6]):
            if runtime_rpy is None:
                with self.lock:
                    current_coords = self.read_coords_locked(robot)
                runtime_rpy = [float(value) for value in current_coords[3:6]]
            coords[3:6] = runtime_rpy[:3]
            rpy_source = "runtime_current"
        return [float(value) for value in coords], runtime_rpy, rpy_source

    def run_coordinate_step(
        self,
        robot: MyCobotDriver,
        step: Dict[str, Any],
        runtime_rpy: Optional[List[float]],
    ) -> tuple[Dict[str, Any], Optional[List[float]]]:
        target, runtime_rpy, rpy_source = self.resolve_step_coords(robot, step, runtime_rpy)
        speed = max(1, min(int(step.get("coordSpeed") or 20), 100))
        mode = 1 if int(step.get("coordMode") or 0) == 1 else 0
        physical_position_tolerance_mm = (
            COORD_PHYSICAL_TOLERANCE_MM
            if mode == 1 else COORD_PHYSICAL_ANGULAR_TOLERANCE_MM
        )
        timeout_s = clamp_float(step.get("timeoutMs", JOINT_MOVE_TIMEOUT_S * 1000.0), 1000, 24000) / 1000.0
        start_coords: Optional[List[float]] = None
        try:
            with self.lock:
                start_coords = self.read_coords_locked(robot)
        except Exception:
            start_coords = None
        try:
            with self.lock:
                current_angles = self.read_angles_locked(robot)
                firmware_angles = robot.solve_inv_kinematics(target, current_angles)
                firmware_fk = robot.angles_to_coords(firmware_angles)
            profile = step.get("toolProfile") or self.tool_profile or {}
            raw_correction = profile.get("tcpCorrectionLocalM") or {}
            correction = [float(raw_correction.get(axis, 0.0)) for axis in ("x", "y", "z")]
            suction_distance = float((profile.get("geometry") or {}).get("flangeToContactM", 0.072))
            tool_id = str(step.get("activeTool") or self.end_effector)
            ik_validation = self._validate_firmware_ik(
                target, firmware_angles, current_angles, firmware_fk, tool_id,
                correction, suction_distance,
            )
            suction_j6_lock = step.get("suctionJ6LockDeg")
            if tool_id == "suction_gripper" and suction_j6_lock is not None:
                runtime_j6_error = abs(
                    ((float(firmware_angles[5]) - float(suction_j6_lock) + 180.0) % 360.0) - 180.0
                )
                ik_validation["suctionJ6LockDeg"] = round(float(suction_j6_lock), 3)
                ik_validation["suctionJ6LockErrorDeg"] = round(runtime_j6_error, 3)
                if runtime_j6_error > SUCTION_J6_LOCK_TOLERANCE_DEG:
                    ik_validation.setdefault("rejectionReasons", []).append(
                        "suction_j6_lock_violation"
                    )
                    ik_validation["ok"] = False
        except Exception as exc:
            ik_validation = {"ok": False, "rejectionReasons": [f"firmware_ik_error: {exc}"]}
        if not ik_validation.get("ok"):
            rejection_reasons = list(ik_validation.get("rejectionReasons") or [])
            failure_reason = (
                "host_ik_unreachable"
                if "host_ik_unreachable" in rejection_reasons else
                "firmware_ik_rejected"
            )
            raise CoordinateMotionError(
                failure_reason,
                target,
                start_coords,
                f"Runtime IK validation rejected state {step.get('stateId') or step.get('name')}",
                details={"ikValidation": ik_validation},
            )
        suction_locked_joint_move = bool(
            tool_id == "suction_gripper"
            and mode == 0
            and step.get("suctionJ6LockDeg") is not None
        )
        with self.lock:
            if suction_locked_joint_move:
                locked_angles = [float(value) for value in firmware_angles[:6]]
                locked_angles[5] = float(step["suctionJ6LockDeg"])
                robot.send_angles(locked_angles, speed)
            else:
                robot.send_coords(target, speed, mode)
        time.sleep(MOTION_COMMAND_FEEDBACK_DELAY_S)
        settle = self.wait_for_motion_stop(
            robot,
            target_coords=target,
            timeout_s=timeout_s,
            position_tolerance_mm=physical_position_tolerance_mm,
            rpy_tolerance_deg=COORD_PHYSICAL_RPY_TOLERANCE_DEG,
        )

        actual_angles: Optional[List[float]] = None
        actual_coords: Optional[List[float]] = None
        try:
            with self.lock:
                actual_angles = self.read_angles_locked(robot)
        except Exception:
            actual_angles = None
        try:
            with self.lock:
                actual_coords = self.read_coords_locked(robot)
        except Exception:
            actual_coords = None

        errors = self.coords_error(
            target, actual_coords,
            physical_position_tolerance_mm,
            COORD_PHYSICAL_RPY_TOLERANCE_DEG,
        )
        record: Dict[str, Any] = {
            "command": "send_angles_j6_locked" if suction_locked_joint_move else "send_coords",
            "speed": speed,
            "coordMode": mode,
            "coordModeName": "linear" if mode == 1 else "angular",
            "toolRpySource": rpy_source,
            "targetCoords": [round(value, 2) for value in target],
            "targetFlangeCoords": [round(value, 2) for value in target],
            "ikValidation": ik_validation,
            **settle,
            **errors,
        }
        if step.get("targetTcpPoseM"):
            record["targetTcpPoseM"] = step["targetTcpPoseM"]
        else:
            target_flange_position = tuple(float(value) / 1000.0 for value in target[:3])
            target_tcp_position, _ = tcp_from_flange(
                target_flange_position, rotation_from_rpy_deg(target[3:6]),
                str(step.get("activeTool") or self.end_effector),
                correction if 'correction' in locals() else None,
                suction_distance if 'suction_distance' in locals() else 0.072,
            )
            record["targetTcpPoseM"] = dict(
                zip(("x", "y", "z"), (round(value, 6) for value in target_tcp_position))
            )
        if step.get("targetFlangePoseM"):
            record["targetFlangePoseM"] = step["targetFlangePoseM"]
        if actual_angles is not None:
            record["actualAngles"] = [round(value, 2) for value in actual_angles]
            if step.get("suctionJ6LockDeg") is not None:
                actual_j6_lock_error = abs(
                    ((float(actual_angles[5]) - float(step["suctionJ6LockDeg"]) + 180.0) % 360.0) - 180.0
                )
                record["suctionJ6LockDeg"] = round(float(step["suctionJ6LockDeg"]), 3)
                record["actualJ6LockErrorDeg"] = round(actual_j6_lock_error, 3)
                if actual_j6_lock_error > SUCTION_J6_EXECUTION_TOLERANCE_DEG:
                    raise CoordinateMotionError(
                        "suction_j6_lock_missed",
                        target,
                        start_coords,
                        f"J6 moved outside the suction lock at {step.get('stateId') or step.get('name')}",
                        details={"motion": record},
                    )
        if actual_coords is not None:
            record["actualCoords"] = [round(value, 2) for value in actual_coords]
        if start_coords is not None:
            move_delta = math.sqrt(sum((float(actual_coords[i]) - float(start_coords[i])) ** 2 for i in range(3))) if actual_coords else None
            record["startCoords"] = [round(value, 2) for value in start_coords]
            if move_delta is not None:
                record["actualMoveDeltaMm"] = round(move_delta, 2)

        target_missed = (
            actual_coords is not None
            and not bool(errors.get("withinTolerance"))
            and settle.get("completion") != "in_position"
        )
        if not settle.get("reached") or target_missed:
            refused = (
                target_missed
                and start_coords is not None
                and actual_coords is not None
                and math.sqrt(sum((float(actual_coords[i]) - float(start_coords[i])) ** 2 for i in range(3))) < 2.0
            )
            controller_error = self.read_controller_error(robot)
            if controller_error is not None:
                record["controllerError"] = controller_error
            self.stop_robot_after_feedback_loss(robot)
            if refused:
                record["failureHints"] = [
                    "firmware refused or could not solve the flange coordinate target",
                    "captured tool RPY may not match the real pick orientation",
                    "tool-offset-adjusted flange target may be outside reachable workspace",
                ]
            raise CoordinateMotionError(
                (
                    "controller_ik_no_solution"
                    if refused and int((controller_error or {}).get("code", 0)) == 32 else
                    "controller_linear_ik_no_solution"
                    if refused and int((controller_error or {}).get("code", 0)) in (33, 34) else
                    "firmware_coordinate_refused_or_unreachable"
                    if refused else
                    "coordinate_target_missed"
                    if target_missed else
                    "coordinate_motion_not_verified"
                ),
                target,
                actual_coords,
                (
                    f"Coordinate move did not verify ({settle.get('completion')}); "
                    f"target {[round(v, 2) for v in target]}"
                ),
                details=record,
            )
        return record, runtime_rpy

    def read_controller_error(self, robot: MyCobotDriver) -> Optional[Dict[str, Any]]:
        if not hasattr(robot, "get_error_information"):
            return None
        try:
            with self.lock:
                code = robot.get_error_information()
        except Exception as exc:
            return {"code": None, "label": "controller_error_read_failed", "readError": str(exc)}
        if code is None:
            return None
        numeric_code = int(code)
        return {
            "code": numeric_code,
            "label": CONTROLLER_ERROR_LABELS.get(numeric_code, "unknown_controller_error"),
        }

    def stop_robot_after_feedback_loss(self, robot: MyCobotDriver) -> None:
        try:
            with self.lock:
                robot.stop()
        except Exception:
            pass

    def recover_motion_feedback(
        self,
        robot: MyCobotDriver,
        target: List[float],
        prior_feedback_misses: int = 0,
        initial_error: str = "",
    ) -> Dict[str, Any]:
        started = time.monotonic()
        deadline = started + MOTION_FEEDBACK_RECOVERY_WINDOW_S
        attempts = 0
        recovery_misses = 0
        drained_bytes = 0
        discarded_frames = 0
        last_error = initial_error

        while time.monotonic() < deadline:
            self.check_abort()
            attempts += 1
            try:
                with self.lock:
                    if hasattr(robot, "drain_input"):
                        drained_bytes += robot.drain_input()
                    angles = robot.get_angles()
                discarded_frames += int(getattr(robot, "last_discarded_frames", 0) or 0)
                drained_bytes += int(getattr(robot, "last_drained_bytes", 0) or 0)
                self.last_angles = angles
                self.last_error = None
                self.last_read_at = time.time()
                error = self.angle_error_deg(target, angles)
                return {
                    "ok": True,
                    "recoveredAfterMiss": True,
                    "recoveryReads": attempts,
                    "feedbackMisses": int(prior_feedback_misses) + recovery_misses,
                    "recoveryMisses": recovery_misses,
                    "drainedBytes": drained_bytes,
                    "discardedFrames": discarded_frames,
                    "actualAngles": [round(value, 2) for value in angles],
                    "errorDeg": round(error, 2),
                    "recoveryElapsedS": round(time.monotonic() - started, 2),
                }
            except PlanAborted:
                raise
            except Exception as exc:
                recovery_misses += 1
                discarded_frames += int(getattr(robot, "last_discarded_frames", 0) or 0)
                drained_bytes += int(getattr(robot, "last_drained_bytes", 0) or 0)
                last_error = str(exc)
                time.sleep(MOTION_FEEDBACK_RECOVERY_DELAY_S)

        actual = list(self.last_angles)
        error = self.angle_error_deg(target, actual)
        self.stop_robot_after_feedback_loss(robot)
        raise MotionProgressError(
            "serial_feedback_lost",
            target,
            actual,
            error,
            (
                f"Lost joint feedback for {MOTION_FEEDBACK_RECOVERY_WINDOW_S:.1f}s "
                f"after {int(prior_feedback_misses) + recovery_misses} missed reads; "
                f"last read error: {last_error}"
            ),
            feedback_misses=int(prior_feedback_misses) + recovery_misses,
            details={
                "recoveryAttempted": True,
                "recoveryReads": attempts,
                "recoveryMisses": recovery_misses,
                "drainedBytes": drained_bytes,
                "discardedFrames": discarded_frames,
                "initialReadError": initial_error,
                "lastReadError": last_error,
            },
        )

    def recover_angle_feedback_locked(
        self,
        robot: MyCobotDriver,
        target: Optional[List[float]] = None,
    ) -> Dict[str, Any]:
        attempts = 0
        feedback_misses = 0
        discarded_frames = 0
        drained_bytes = 0
        last_error = ""
        target_values = list(target or self.last_angles)

        for attempt in range(1, PROGRAM_GRIPPER_RECOVERY_ATTEMPTS + 1):
            self.check_abort()
            attempts = attempt
            if hasattr(robot, "drain_input"):
                drained_bytes += robot.drain_input()
            try:
                angles = robot.get_angles(response_timeout=PROGRAM_GRIPPER_RECOVERY_TIMEOUT_S)
                discarded_frames += int(getattr(robot, "last_discarded_frames", 0) or 0)
                drained_bytes += int(getattr(robot, "last_drained_bytes", 0) or 0)
                self.last_angles = angles
                self.last_error = None
                self.last_read_at = time.time()
                return {
                    "ok": True,
                    "recoveryReads": attempts,
                    "feedbackMisses": feedback_misses,
                    "discardedFrames": discarded_frames,
                    "drainedBytes": drained_bytes,
                    "actualAngles": [round(value, 2) for value in angles],
                }
            except Exception as exc:
                feedback_misses += 1
                discarded_frames += int(getattr(robot, "last_discarded_frames", 0) or 0)
                drained_bytes += int(getattr(robot, "last_drained_bytes", 0) or 0)
                last_error = str(exc)
                time.sleep(PROGRAM_GRIPPER_RECOVERY_DELAY_S)

        actual = list(self.last_angles)
        error = self.angle_error_deg(target_values, actual)
        raise MotionProgressError(
            "feedback_recovery_failed",
            target_values,
            actual,
            error,
            f"Could not recover joint feedback after gripper action: {last_error}",
            feedback_misses=feedback_misses,
        )

    def read_motion_feedback(
        self,
        robot: MyCobotDriver,
        target: List[float],
        missed_since: Optional[float],
        feedback_misses: int,
    ) -> tuple[Optional[List[float]], Optional[float], int, Optional[Dict[str, Any]]]:
        try:
            with self.lock:
                angles = self.read_angles_locked(robot)
            return angles, None, feedback_misses, None
        except Exception as exc:
            now = time.monotonic()
            feedback_misses += 1
            if missed_since is None:
                missed_since = now
            if now - missed_since >= JOINT_FEEDBACK_MISS_TIMEOUT_S:
                recovery = self.recover_motion_feedback(
                    robot,
                    target,
                    prior_feedback_misses=feedback_misses,
                    initial_error=str(exc),
                )
                angles = [float(value) for value in recovery["actualAngles"]]
                return angles, None, int(recovery.get("feedbackMisses") or feedback_misses), recovery
            return None, missed_since, feedback_misses, None

    def check_abort(self) -> None:
        if self.abort_event.is_set():
            raise PlanAborted("Physical plan stopped by operator")

    @staticmethod
    def recovery_summary(recoveries: List[Dict[str, Any]]) -> Dict[str, Any]:
        if not recoveries:
            return {}
        return {
            "recoveredAfterMiss": True,
            "recoveryReads": sum(int(item.get("recoveryReads") or 0) for item in recoveries),
            "recoveryMisses": sum(int(item.get("recoveryMisses") or 0) for item in recoveries),
            "drainedBytes": sum(int(item.get("drainedBytes") or 0) for item in recoveries),
            "discardedFrames": sum(int(item.get("discardedFrames") or 0) for item in recoveries),
            "recoveryEvents": recoveries,
        }

    def wait_for_joint_target(
        self,
        robot: MyCobotDriver,
        target: List[float],
        tolerance_deg: float,
        timeout_s: float,
    ) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        started_at = time.monotonic()
        stable_count = 0
        settled_count = 0
        reads = 0
        last_error = 999.0
        last_angles = self.last_angles
        previous_angles: Optional[List[float]] = None
        last_delta = 999.0
        missed_since: Optional[float] = None
        feedback_misses = 0
        recoveries: List[Dict[str, Any]] = []

        while True:
            now = time.monotonic()
            if now >= deadline and not (
                missed_since is not None
                and now - missed_since < JOINT_FEEDBACK_MISS_TIMEOUT_S
            ):
                break
            self.check_abort()
            angles, missed_since, feedback_misses, recovery = self.read_motion_feedback(
                robot, target, missed_since, feedback_misses
            )
            if angles is None:
                time.sleep(JOINT_FEEDBACK_POLL_S)
                continue
            if recovery:
                recoveries.append(recovery)
            last_angles = angles
            reads += 1

            last_error = self.angle_error_deg(target, last_angles)
            last_delta = self.angle_error_deg(previous_angles, last_angles) if previous_angles else 999.0
            previous_angles = list(last_angles)
            if recovery and last_error <= tolerance_deg:
                return {
                    "reached": True,
                    "completion": "target_recovered",
                    "reads": reads,
                    "feedbackMisses": feedback_misses,
                    "errorDeg": round(last_error, 2),
                    "settledDeltaDeg": round(last_delta, 2),
                    "actualAngles": [round(value, 2) for value in last_angles],
                    **self.recovery_summary(recoveries),
                }
            if (
                recovery
                and time.monotonic() - started_at >= JOINT_MIN_MOVE_WAIT_S
                and last_error <= JOINT_TARGET_SOFT_TOLERANCE_DEG
            ):
                return {
                    "reached": True,
                    "completion": "settled_near_target_after_recovery",
                    "reads": reads,
                    "feedbackMisses": feedback_misses,
                    "errorDeg": round(last_error, 2),
                    "settledDeltaDeg": round(last_delta, 2),
                    "actualAngles": [round(value, 2) for value in last_angles],
                    **self.recovery_summary(recoveries),
                }
            if last_error <= tolerance_deg:
                stable_count += 1
                if stable_count >= JOINT_TARGET_STABLE_SAMPLES:
                    return {
                        "reached": True,
                        "completion": "target_reached",
                        "reads": reads,
                        "feedbackMisses": feedback_misses,
                        "errorDeg": round(last_error, 2),
                        "settledDeltaDeg": round(last_delta, 2),
                        "actualAngles": [round(value, 2) for value in last_angles],
                        **self.recovery_summary(recoveries),
                    }
            else:
                stable_count = 0
                if (
                    time.monotonic() - started_at >= JOINT_MIN_MOVE_WAIT_S
                    and last_error <= JOINT_TARGET_SOFT_TOLERANCE_DEG
                    and last_delta <= JOINT_SETTLED_DELTA_DEG
                ):
                    settled_count += 1
                    if settled_count >= JOINT_TARGET_STABLE_SAMPLES:
                        return {
                            "reached": True,
                            "completion": "settled_near_target",
                            "reads": reads,
                            "feedbackMisses": feedback_misses,
                            "errorDeg": round(last_error, 2),
                            "settledDeltaDeg": round(last_delta, 2),
                            "actualAngles": [round(value, 2) for value in last_angles],
                            **self.recovery_summary(recoveries),
                        }
                else:
                    settled_count = 0
            time.sleep(JOINT_FEEDBACK_POLL_S)

        if last_error <= JOINT_TARGET_SOFT_TOLERANCE_DEG:
            return {
                "reached": True,
                "completion": "timeout_soft_reached",
                "reads": reads,
                "feedbackMisses": feedback_misses,
                "errorDeg": round(last_error, 2),
                "settledDeltaDeg": round(last_delta, 2),
                "actualAngles": [round(value, 2) for value in last_angles],
                **self.recovery_summary(recoveries),
            }
        if missed_since is not None:
            recovery = self.recover_motion_feedback(
                robot,
                target,
                prior_feedback_misses=feedback_misses,
                initial_error="deadline reached while feedback was missing",
            )
            recoveries.append(recovery)
            last_angles = [float(value) for value in recovery["actualAngles"]]
            last_error = self.angle_error_deg(target, last_angles)
            if last_error <= tolerance_deg:
                return {
                    "reached": True,
                    "completion": "target_recovered",
                    "reads": reads,
                    "feedbackMisses": int(recovery.get("feedbackMisses") or feedback_misses),
                    "errorDeg": round(last_error, 2),
                    "settledDeltaDeg": round(last_delta, 2),
                    "actualAngles": [round(value, 2) for value in last_angles],
                    **self.recovery_summary(recoveries),
                }
            if last_error <= JOINT_TARGET_SOFT_TOLERANCE_DEG:
                return {
                    "reached": True,
                    "completion": "settled_near_target_after_recovery",
                    "reads": reads,
                    "feedbackMisses": int(recovery.get("feedbackMisses") or feedback_misses),
                    "errorDeg": round(last_error, 2),
                    "settledDeltaDeg": round(last_delta, 2),
                    "actualAngles": [round(value, 2) for value in last_angles],
                    **self.recovery_summary(recoveries),
                }

        raise MotionProgressError(
            "motion_unreachable_or_stalled",
            target,
            last_angles,
            last_error,
            f"Timed out waiting for joint target; max error {last_error:.2f} deg, "
            f"last delta {last_delta:.2f} deg, target {[round(value, 2) for value in target]}, "
            f"actual {[round(value, 2) for value in last_angles]}",
            feedback_misses=feedback_misses,
            details=self.recovery_summary(recoveries),
        )

    def wait_for_motion_stop(
        self,
        robot: MyCobotDriver,
        target_coords: Optional[List[float]] = None,
        timeout_s: float = JOINT_MOVE_TIMEOUT_S,
        position_tolerance_mm: float = COORD_PHYSICAL_TOLERANCE_MM,
        rpy_tolerance_deg: float = COORD_PHYSICAL_RPY_TOLERANCE_DEG,
    ) -> Dict[str, Any]:
        """Wait for a firmware Cartesian move to finish, staying abortable.

        Releases the serial lock between polls (so Stop still works) and tolerates
        transient feedback misses without killing the plan.
        """
        started = time.monotonic()
        deadline = started + timeout_s
        reads = 0
        idle = 0
        stable_near_target = 0
        misses = 0
        missed_since: Optional[float] = None
        last_stopped_coords: Optional[List[float]] = None
        last_coord_error: Optional[Dict[str, Any]] = None
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            self.check_abort()
            try:
                with self.lock:
                    moving = robot.is_moving()
                    in_pos = (
                        robot.is_in_position(target_coords, 1)
                        if target_coords is not None
                        else None
                    )
                reads += 1
                missed_since = None
            except Exception:
                misses += 1
                if missed_since is None:
                    missed_since = now
                if now - missed_since >= JOINT_FEEDBACK_MISS_TIMEOUT_S:
                    return {
                        "reached": False, "completion": "feedback_lost",
                        "reads": reads, "feedbackMisses": misses,
                    }
                time.sleep(JOINT_FEEDBACK_POLL_S)
                continue
            elapsed = now - started
            if in_pos == 1 and elapsed >= JOINT_MIN_MOVE_WAIT_S:
                return {
                    "reached": True, "completion": "in_position",
                    "reads": reads, "feedbackMisses": misses,
                }
            if moving == 0:
                idle += 1
                if elapsed >= JOINT_MIN_MOVE_WAIT_S and target_coords is not None:
                    try:
                        with self.lock:
                            stopped_coords = self.read_coords_locked(robot)
                        coord_delta = (
                            max(abs(float(stopped_coords[i]) - float(last_stopped_coords[i])) for i in range(3))
                            if last_stopped_coords is not None else float("inf")
                        )
                        last_coord_error = self.coords_error(
                            target_coords,
                            stopped_coords,
                            position_tolerance_mm,
                            rpy_tolerance_deg,
                        )
                        last_stopped_coords = stopped_coords
                        if last_coord_error["withinTolerance"] and coord_delta <= COORD_SETTLED_DELTA_MM:
                            stable_near_target += 1
                        else:
                            stable_near_target = 0
                        if stable_near_target >= COORD_STOP_STABLE_SAMPLES:
                            return {
                                "reached": True,
                                "completion": "settled_near_target",
                                "reads": reads,
                                "feedbackMisses": misses,
                                "settledSamples": stable_near_target,
                                "settledDeltaMm": round(coord_delta, 2),
                                "settledCoords": [round(float(value), 2) for value in stopped_coords],
                                **last_coord_error,
                            }
                    except Exception:
                        misses += 1
                elif idle >= 2 and elapsed >= JOINT_MIN_MOVE_WAIT_S:
                    return {
                        "reached": True, "completion": "stopped_unverified",
                        "reads": reads, "feedbackMisses": misses,
                    }
                # Do not interpret the first two idle polls as completion: the
                # controller can briefly report idle while a coordinate command
                # is being accepted. Outside-tolerance stopped feedback is
                # sampled several times before returning a definitive failure.
                if (
                    idle >= max(6, COORD_STOP_STABLE_SAMPLES + 2)
                    and last_coord_error is not None
                    and not last_coord_error["withinTolerance"]
                ):
                    return {
                        "reached": False,
                        "completion": "stopped_outside_tolerance",
                        "reads": reads,
                        "feedbackMisses": misses,
                        "settledSamples": stable_near_target,
                        "settledCoords": (
                            [round(float(value), 2) for value in last_stopped_coords]
                            if last_stopped_coords is not None else None
                        ),
                        **last_coord_error,
                    }
            elif moving == 1:
                idle = 0
                stable_near_target = 0
                last_stopped_coords = None
            time.sleep(JOINT_FEEDBACK_POLL_S)
        return {
            "reached": False, "completion": "timeout",
            "reads": reads, "feedbackMisses": misses,
        }

    def wait_for_gripper_state(
        self,
        robot: MyCobotDriver,
        action: str,
        timeout_s: float = GRIPPER_ACTION_TIMEOUT_S,
    ) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        reads = 0
        saw_feedback = False
        last_moving: Optional[int] = None
        last_value: Optional[int] = None
        idle_count = 0
        value_stable_count = 0

        while time.monotonic() < deadline:
            self.check_abort()
            try:
                with self.lock:
                    moving = robot.is_gripper_moving()
                reads += 1
                saw_feedback = saw_feedback or moving is not None
                last_moving = moving
                if moving == 0:
                    idle_count += 1
                    if idle_count >= 2:
                        try:
                            with self.lock:
                                last_value = robot.get_gripper_value()
                        except Exception:
                            last_value = None
                        return {
                            "feedback": "idle",
                            "action": action,
                            "reads": reads,
                            "moving": last_moving,
                            "value": last_value,
                        }
                elif moving == 1:
                    idle_count = 0
            except PlanAborted:
                raise
            except Exception:
                if saw_feedback:
                    raise
                moving = None

            if moving is None:
                try:
                    with self.lock:
                        value = robot.get_gripper_value()
                    reads += 1
                    saw_feedback = saw_feedback or value is not None
                    if value is not None and last_value is not None and abs(value - last_value) <= 1:
                        value_stable_count += 1
                    else:
                        value_stable_count = 0
                    last_value = value
                    if value_stable_count >= 3:
                        return {
                            "feedback": "value_stable",
                            "action": action,
                            "reads": reads,
                            "moving": last_moving,
                            "value": last_value,
                        }
                except Exception:
                    if saw_feedback:
                        raise

            time.sleep(GRIPPER_FEEDBACK_POLL_S)

        return {
            "feedback": "timeout_guard" if saw_feedback else "unavailable_timeout_guard",
            "action": action,
            "reads": reads,
            "moving": last_moving,
            "value": last_value,
        }

    def run_gripper_action(
        self,
        robot: MyCobotDriver,
        action: str,
        program_mode: bool = False,
    ) -> Dict[str, Any]:
        # flag: 0 = open, 1 = close (adaptive auto-grip). Speeds match prior tuning.
        actions = {
            "open_before_approach": (0, 45),
            "auto_grip": (1, 35),
            "open_at_drop": (0, 35),
        }
        if action not in actions:
            raise ValueError(f"Unknown gripper action: {action}")
        flag, speed = actions[action]
        if self.end_effector == "suction_gripper":
            suction_enabled = action == "auto_grip"
            with self.lock:
                suction = robot.suction_on() if suction_enabled else robot.suction_off()
            if suction_enabled:
                # Contact motion has already verified before after-arrival
                # actions run. Give the base-mounted pump time to establish
                # vacuum before the next (lift) state can begin.
                time.sleep(0.6)
            return {
                "action": action,
                "endEffector": self.end_effector,
                "suction": "on" if suction_enabled else "off",
                **suction,
                "feedback": "suction_commanded",
                "pumpSettleS": 0.6 if suction_enabled else 0.0,
                "discardedFrames": int(getattr(robot, "last_discarded_frames", 0) or 0),
                "drainedBytes": int(getattr(robot, "last_drained_bytes", 0) or 0),
            }
        with self.lock:
            robot.set_gripper_state(flag, speed)
        # Always wait for the jaws to actually finish moving (over the robust
        # pymycobot transport) instead of a blind sleep, so the next motion -
        # e.g. the lift right after a grasp - only starts once the gripper has
        # settled. program_mode just keeps a shorter ceiling so a missed grip
        # can't stall a running plan.
        timeout_s = PROGRAM_GRIPPER_TIMEOUT_S if program_mode else GRIPPER_ACTION_TIMEOUT_S
        result = self.wait_for_gripper_state(robot, action, timeout_s=timeout_s)
        return {
            "action": action,
            "uartFlag": flag,
            "speed": speed,
            "discardedFrames": int(getattr(robot, "last_discarded_frames", 0) or 0),
            "drainedBytes": int(getattr(robot, "last_drained_bytes", 0) or 0),
            **result,
        }

    @staticmethod
    def _validate_angle_list(state_id: Any, angles: Any) -> Optional[str]:
        try:
            values = [float(value) for value in angles]
        except (TypeError, ValueError):
            return f"State {state_id} has non-numeric joint angles."
        if len(values) != 6:
            return f"State {state_id} needs exactly 6 joint angles."
        for joint, degrees in enumerate(values, start=1):
            try:
                validate_joint_angle(joint, degrees)
            except ValueError as exc:
                return f"State {state_id}: {exc}"
        return None

    @classmethod
    def validate_plan_steps(cls, steps: List[Dict[str, Any]]) -> Optional[str]:
        for step in steps:
            state_id = step.get("stateId") or step.get("name") or "unknown"
            if step.get("coordsMm") is not None:
                coords = step.get("coordsMm")
                if not isinstance(coords, list) or len(coords) != 6:
                    return f"State {state_id} needs coordsMm [x,y,z,rx,ry,rz]."
                for index, value in enumerate(coords):
                    if value is None and index >= 3:
                        continue
                    try:
                        float(value)
                    except (TypeError, ValueError):
                        return f"State {state_id} has non-numeric coordinate values."
                mode = int(step.get("coordMode") or 0)
                if mode not in (0, 1):
                    return f"State {state_id} has invalid coordMode {mode}; expected 0 or 1."
                bounds_errors = validate_coordinate_bounds(
                    coords,
                    str(state_id),
                    allow_missing_rpy=True,
                    xy_margin_mm=generated_coordinate_xy_margin_mm(step),
                )
                if bounds_errors:
                    return bounds_errors[0].get("message") or f"State {state_id} has invalid coordinate bounds."
                continue
            if step.get("jointTargetDeg") is not None:
                error = cls._validate_angle_list(state_id, step.get("jointTargetDeg"))
                if error:
                    return error
                captured = step.get("capturedFlangeCoordsMmDeg")
                if captured is not None:
                    bounds_errors = validate_coordinate_bounds(captured, str(state_id), allow_missing_rpy=False)
                    if bounds_errors:
                        return bounds_errors[0].get("message") or f"State {state_id} has invalid captured coordinates."
                continue
            if step.get("waitMs") is not None:
                try:
                    wait_ms = int(step.get("waitMs"))
                except (TypeError, ValueError):
                    return f"State {state_id} has an invalid wait duration."
                if wait_ms < 50 or wait_ms > 600000:
                    return f"State {state_id} wait must be between 50 ms and 600000 ms."
                continue
            if step.get("robotCommand") or step.get("gripperAction"):
                continue
            return f"State {state_id} must be a coordinate, robot command, or gripper action state."
        return None

    @staticmethod
    def coordinate_preflight_error(steps: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        for step in steps:
            coords = step.get("coordsMm")
            if coords is None:
                continue
            state_id = str(step.get("stateId") or step.get("name") or "unknown")
            errors = validate_coordinate_bounds(
                coords,
                state_id,
                allow_missing_rpy=True,
                xy_margin_mm=generated_coordinate_xy_margin_mm(step),
            )
            if errors:
                first = errors[0]
                return {
                    "error": first.get("message") or first.get("error") or "Coordinate preflight failed.",
                    "failedState": state_id,
                    "coordinatePreflight": {
                        "ok": False,
                        "errors": errors,
                    },
                }
        return None

    def execute_pick_plan(self, plan: Dict[str, Any], confirm: Any) -> Dict[str, Any]:
        if confirm != "RUN_PHYSICAL_PICK":
            return {"ok": False, "error": "Physical execution requires confirm='RUN_PHYSICAL_PICK'."}
        steps = plan.get("steps") or []
        if not steps:
            return {"ok": False, "error": "No plan steps were provided."}
        if plan.get("mode") != "coordinate_program":
            return {"ok": False, "error": "This is an old non-coordinate plan. Plan again before running."}
        preflight = self.coordinate_preflight_error(steps)
        if preflight:
            return {
                "ok": False,
                "executedSteps": [],
                **preflight,
                **self.status_locked(),
            }
        if bool(plan.get("requiresCapturedToolRpy")):
            return {
                "ok": False,
                "error": "Capture tool orientation before running coordinate pick/place.",
                "failedState": None,
                "executedSteps": [],
                **self.status_locked(),
            }
        if not bool(plan.get("physicalReady", True)):
            return {
                "ok": False,
                "error": "Plan is not ready for physical execution. Plan again and resolve the reported preflight issue.",
                "failedState": None,
                "executedSteps": [],
                **self.status_locked(),
            }
        if not bool((plan.get("coordinatePreview") or {}).get("ok")):
            return {
                "ok": False,
                "error": "A complete validated coordinate preview is required before physical execution.",
                "executedSteps": [],
                **self.status_locked(),
            }
        for step in steps:
            coords = step.get("coordsMm")
            if isinstance(coords, list) and len(coords) >= 6 and any(value is None for value in coords[3:6]):
                return {
                    "ok": False,
                    "error": "Capture tool orientation before running coordinate pick/place.",
                    "failedState": step.get("stateId") or step.get("name"),
                    "executedSteps": [],
                    **self.status_locked(),
                }
        for step in steps:
            if (
                step.get("trajectory")
                and step.get("jointTargetDeg") is None
                and step.get("coordsMm") is None
            ) or (
                step.get("jointAngles")
                and not step.get("robotCommand")
                and not step.get("jointTargetDeg")
            ):
                return {
                    "ok": False,
                    "error": "This plan contains legacy joint/trajectory motion. Plan again to generate coordinate moves.",
                }

        # Reject the whole plan before any motion instead of failing mid-sequence.
        plan_error = self.validate_plan_steps(steps)
        if plan_error:
            return {"ok": False, "error": plan_error}

        # Firmware IK depends on the current joint branch. Re-run the complete
        # preview immediately before acquiring the execution flag so stale
        # plans cannot authorize motion from a different robot pose.
        angle_result = self.get_angles()
        if not angle_result.get("ok"):
            return {"ok": False, "error": f"Could not read current angles for IK revalidation: {angle_result.get('error')}"}
        self.add_coordinate_preview(plan, angle_result.get("angles") or self.last_angles)
        fresh_preview = plan.get("coordinatePreview") or {}
        if not fresh_preview.get("ok"):
            return {
                "ok": False,
                "error": f"Fresh IK validation failed: {fresh_preview.get('error') or 'incomplete path'}",
                "coordinatePreview": fresh_preview,
                "executedSteps": [],
                **self.status_locked(),
            }

        with self.lock:
            if self.executing:
                return {"ok": False, "error": "A physical plan is already running.", **self.status_locked()}
            if self.jog_session:
                return {"ok": False, "error": "Stop jogging before running a physical program.", **self.status_locked()}
            try:
                robot = self.get_robot_locked()
            except Exception as exc:
                self.last_error = str(exc)
                return {"ok": False, "error": str(exc), **self.status_locked()}
            self.executing = True
            self.execution_progress = None
            self.abort_event.clear()

        completed: List[Dict[str, Any]] = []
        failed_state: Optional[str] = None
        try:
            last_angles: Optional[List[float]] = None
            runtime_tool_rpy: Optional[List[float]] = None
            current_record: Optional[Dict[str, Any]] = None
            if any(
                isinstance(step.get("coordsMm"), list)
                and len(step["coordsMm"]) >= 6
                and any(value is None for value in step["coordsMm"][3:6])
                for step in steps
            ):
                with self.lock:
                    current_coords = self.read_coords_locked(robot)
                runtime_tool_rpy = [float(value) for value in current_coords[3:6]]

            for step in steps:
                self.check_abort()
                name = step.get("name")
                state_id = step.get("stateId") or name
                failed_state = state_id
                with self.lock:
                    self.execution_progress = {
                        "stateId": state_id,
                        "sourceStepId": step.get("sourceStepId"),
                        "sourceIteration": step.get("sourceIteration", 1),
                    }
                gripper_action = step.get("gripperAction")
                if gripper_action is None:
                    if name == "observe" and step.get("gripper") == "open":
                        gripper_action = "open_before_approach"
                    elif name == "auto_grip":
                        gripper_action = "auto_grip"
                    elif name == "release_gripper":
                        gripper_action = "open_at_drop"
                gripper_timing = step.get("gripperActionTiming") or (
                    "before_move" if gripper_action == "open_before_approach" else "after_arrival"
                )
                timeout_s = clamp_float(step.get("timeoutMs", JOINT_MOVE_TIMEOUT_S * 1000.0), 1000, 20000) / 1000.0

                state_record: Dict[str, Any] = {
                    "stateId": state_id,
                    "name": name,
                    "sourceStepId": step.get("sourceStepId"),
                    "sourceIteration": step.get("sourceIteration", 1),
                }
                current_record = state_record
                if step.get("releaseObjectId"):
                    state_record["releaseObjectId"] = step["releaseObjectId"]
                    if step.get("placedPosition"):
                        state_record["placedPosition"] = step["placedPosition"]

                if gripper_action and gripper_timing == "before_move":
                    state_record["gripper"] = self.run_gripper_action(
                        robot,
                        gripper_action,
                        program_mode=True,
                    )

                if step.get("robotCommand") == "home":
                    speed = int(clamp_float(step.get("speed", 15), 1, 100))
                    with self.lock:
                        robot.send_angles(HOME_ANGLES, speed)
                    time.sleep(MOTION_COMMAND_FEEDBACK_DELAY_S)
                    feedback = self.wait_for_joint_target(
                        robot,
                        HOME_ANGLES,
                        JOINT_TARGET_TOLERANCE_DEG,
                        timeout_s,
                    )
                    state_record["motion"] = {
                        "command": "send_angles",
                        "robotCommand": "home",
                        "speed": speed,
                        "targetAngles": [round(value, 2) for value in HOME_ANGLES],
                        **feedback,
                    }
                    last_angles = feedback.get("actualAngles") or self.last_angles
                elif step.get("jointTargetDeg") is not None:
                    target_angles = [float(value) for value in step["jointTargetDeg"]]
                    speed = int(clamp_float(step.get("speed", 20), 1, 100))
                    with self.lock:
                        robot.send_angles(target_angles, speed)
                    time.sleep(MOTION_COMMAND_FEEDBACK_DELAY_S)
                    feedback = self.wait_for_joint_target(
                        robot,
                        target_angles,
                        JOINT_TARGET_TOLERANCE_DEG,
                        timeout_s,
                    )
                    state_record["motion"] = {
                        "command": "send_angles",
                        "robotCommand": "joint_move",
                        "speed": speed,
                        "targetAngles": [round(value, 3) for value in target_angles],
                        **feedback,
                    }
                    last_angles = feedback.get("actualAngles") or self.last_angles
                elif step.get("coordsMm") is not None:
                    motion, runtime_tool_rpy = self.run_coordinate_step(robot, step, runtime_tool_rpy)
                    state_record["motion"] = motion
                    if motion.get("actualAngles"):
                        last_angles = [float(value) for value in motion["actualAngles"]]
                    else:
                        last_angles = list(self.last_angles)
                elif step.get("waitMs") is not None:
                    wait_seconds = float(step["waitMs"]) / 1000.0
                    deadline = time.monotonic() + wait_seconds
                    while time.monotonic() < deadline:
                        self.check_abort()
                        time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
                    state_record["motion"] = {
                        "command": "wait",
                        "durationMs": int(step["waitMs"]),
                        "completion": "wait_complete",
                    }
                elif not gripper_action:
                    raise ValueError(
                        f"Coordinate program state '{state_id}' has no coordsMm, robotCommand, or gripper action."
                    )

                if gripper_action and gripper_timing == "after_arrival":
                    state_record["gripper"] = self.run_gripper_action(
                        robot,
                        gripper_action,
                        program_mode=True,
                    )
                    if gripper_action == "auto_grip":
                        with self.lock:
                            state_record["feedbackRecovery"] = self.recover_angle_feedback_locked(
                                robot,
                                last_angles,
                            )

                completed.append(state_record)

            with self.lock:
                self.executing = False
                self.execution_progress = None
                self.last_error = None
                return {"ok": True, "executedSteps": completed, **self.status_locked()}
        except PlanAborted as exc:
            # The stop/release command already reached the robot; keep the connection open.
            with self.lock:
                self.executing = False
                self.execution_progress = None
                self.last_error = str(exc)
                return {
                    "ok": False,
                    "aborted": True,
                    "error": str(exc),
                    "failedState": failed_state,
                    "executedSteps": completed,
                    **self.status_locked(),
                }
        except MotionProgressError as exc:
            if current_record is not None and current_record not in completed:
                current_record["motion"] = exc.result()
                completed.append(current_record)
            with self.lock:
                self.executing = False
                self.execution_progress = None
                self.last_error = str(exc)
                return {
                    "ok": False,
                    "error": str(exc),
                    "failedState": failed_state,
                    "executedSteps": completed,
                    **self.status_locked(),
                }
        except CoordinateMotionError as exc:
            if current_record is not None and current_record not in completed:
                current_record["motion"] = exc.result()
                completed.append(current_record)
            with self.lock:
                self.executing = False
                self.execution_progress = None
                self.last_error = str(exc)
                return {
                    "ok": False,
                    "error": str(exc),
                    "failedState": failed_state,
                    "executedSteps": completed,
                    **self.status_locked(),
                }
        except SerialException as exc:
            with self.lock:
                self.executing = False
                self.execution_progress = None
                self.close_locked()
                self.last_error = str(exc)
                return {
                    "ok": False,
                    "error": str(exc),
                    "failedState": failed_state,
                    "executedSteps": completed,
                    **self.status_locked(),
                }
        except Exception as exc:
            with self.lock:
                self.executing = False
                self.execution_progress = None
                self.last_error = str(exc)
                return {
                    "ok": False,
                    "error": str(exc),
                    "failedState": failed_state,
                    "executedSteps": completed,
                    **self.status_locked(),
                }
        finally:
            with self.lock:
                self.executing = False
                self.execution_progress = None
                self.abort_event.clear()


class ProductionProgramRuntime:
    """Server-owned, fail-stop runner for validated repeatable programs."""

    def __init__(self, scene: Workcell, service: RobotService) -> None:
        self.scene = scene
        self.service = service
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.pending_external_trigger = False
        self.session: Dict[str, Any] = self._empty_status()

    @staticmethod
    def _empty_status() -> Dict[str, Any]:
        return {
            "ok": True, "state": "disarmed", "programId": None,
            "programName": None, "mode": None, "cycleCount": 0,
            "maxCycles": None, "activeCommand": None, "lastError": None,
            "pendingTrigger": False, "armedAt": None, "updatedAt": time.time(),
        }

    def status(self) -> Dict[str, Any]:
        with self.lock:
            return deepcopy(self.session)

    def _set_state(self, state: str, **updates: Any) -> None:
        with self.lock:
            # Stop is authoritative. A worker already between loop checks must
            # never overwrite the disarmed state with validating/running.
            if self.stop_event.is_set() and state != "disarmed":
                return
            self.session.update({"state": state, "updatedAt": time.time(), **updates})

    def arm(self, program_id: str, confirm: Any, speed_override_pct: Any = 100) -> Dict[str, Any]:
        if confirm != PHYSICAL_CONFIRM_TOKEN:
            return {"ok": False, "error": "Explicit physical-run confirmation is required."}
        with self.lock:
            if self.thread and self.thread.is_alive():
                return {"ok": False, "error": "A production program is already armed.", **self.session}
            with self.scene.lock:
                program = deepcopy(self.scene.programs.get(str(program_id)))
            if program is None:
                return {"ok": False, "error": f"Program '{program_id}' was not found."}
            cache_error = self.scene.compiled_cycle_error(program)
            if cache_error:
                return {"ok": False, "error": cache_error}
            policy = program.get("runPolicy") or self.scene._normalized_run_policy(program)
            mode = str(policy.get("mode") or "finite")
            trigger_part_id = str(policy.get("triggerPartId") or "") or None
            if mode == "object_triggered" and not trigger_part_id:
                return {"ok": False, "error": "Object-triggered mode requires a registered trigger part."}
            if trigger_part_id and trigger_part_id not in self.scene.registered_parts:
                return {"ok": False, "error": "The configured trigger part is no longer registered."}
            self.stop_event.clear()
            self.pending_external_trigger = False
            maximum = policy.get("maxCycles")
            if mode == "finite":
                maximum = int(policy.get("cycleCount") or program.get("repeatCount") or 1)
            self.session = {
                **self._empty_status(),
                "state": "armed", "programId": program["id"], "programName": program["name"],
                "mode": mode, "maxCycles": maximum, "triggerPartId": trigger_part_id,
                "speedOverridePct": clamp_float(speed_override_pct, 1, 100),
                "armedAt": time.time(), "updatedAt": time.time(),
            }
            self.thread = threading.Thread(
                target=self._run, args=(program,), name="production-program", daemon=True
            )
            self.thread.start()
            return deepcopy(self.session)

    def trigger(self) -> Dict[str, Any]:
        with self.lock:
            if self.session.get("state") in ("disarmed", "completed", "faulted"):
                return {"ok": False, "error": "No external-triggered program is armed.", **self.session}
            if self.session.get("mode") != "external_triggered":
                return {"ok": False, "error": "The armed program is not computer-triggered.", **self.session}
            # Deliberately one-deep: repeated computer events are coalesced.
            self.pending_external_trigger = True
            self.session["pendingTrigger"] = True
            self.session["updatedAt"] = time.time()
            return deepcopy(self.session)

    def stop(self) -> Dict[str, Any]:
        self.stop_event.set()
        with self.lock:
            running = self.session.get("state") in ("validating", "running")
        if running:
            # The same fail-safe stop path used by the dashboard interrupts
            # the motion loop; it is never invoked by offline tests unless a
            # fake service explicitly exercises it.
            try:
                self.service.command("stop")
            except Exception:
                pass
        self._set_state("disarmed", pendingTrigger=False, activeCommand=None)
        return self.status()

    def shutdown(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _object_trigger_ready(
        self, program: Dict[str, Any], absent_since: Optional[float], stable: int,
        last_observation: Optional[float],
    ):
        policy = program.get("runPolicy") or {}
        part_id = str(policy.get("triggerPartId") or "")
        with self.scene.lock:
            part = deepcopy(self.scene.parts.get(part_id))
        visible = bool(part and self.scene.tag_pose_is_fresh(part))
        if not visible:
            absent_since = absent_since or time.monotonic()
            return False, absent_since, 0, None
        if absent_since is not None:
            rearm_s = float(policy.get("rearmAbsentMs") or 1000) / 1000.0
            if time.monotonic() - absent_since < rearm_s:
                return False, absent_since, 0, None
            absent_since = None
        expected_surface = policy.get("expectedSurfaceId")
        if expected_surface and part.get("supportSurfaceId") != expected_surface:
            return False, absent_since, 0, None
        observation = float(part.get("lastLocalizedAt") or part.get("lastSeenAt") or 0.0)
        if observation <= 0.0 or observation == last_observation:
            return False, absent_since, stable, last_observation
        stable += 1
        return (
            stable >= int(policy.get("stableFrames") or 3),
            absent_since, stable, observation,
        )

    def _fresh_cycle_plan(self, program: Dict[str, Any]) -> Dict[str, Any]:
        with self.scene.lock:
            current_program = deepcopy(self.scene.programs.get(str(program.get("id") or "")))
        if current_program is None:
            return {"ok": False, "error": "The armed program was deleted."}
        cache_error = self.scene.compiled_cycle_error(current_program)
        if cache_error:
            return {"ok": False, "error": cache_error}
        program = current_program
        compiled = program.get("compiledCycle") or {}
        policy = program.get("runPolicy") or {}
        status = self.service.status()
        start_angles = [float(value) for value in (status.get("lastAngles") or HOME_ANGLES)]
        if str(policy.get("mode")) == "object_triggered":
            anchor = compiled.get("triggerAnchor") or {}
            part_id = str(policy.get("triggerPartId") or anchor.get("objectId") or "")
            with self.scene.lock:
                current = deepcopy(self.scene.parts.get(part_id))
            if current is None:
                return {"ok": False, "error": "The trigger part is no longer visible."}
            if current.get("supportSurfaceId") != anchor.get("supportSurfaceId"):
                return {"ok": False, "error": "The trigger part changed support surfaces; validate again."}
            anchor_position = anchor.get("position") or {}
            current_position = current.get("position") or {}
            displacement = math.hypot(
                float(current_position.get("x", 0.0)) - float(anchor_position.get("x", 0.0)),
                float(current_position.get("y", 0.0)) - float(anchor_position.get("y", 0.0)),
            )
            yaw_delta = abs(((float(current.get("orientationDeg") or 0.0) - float(anchor.get("orientationDeg") or 0.0) + 180.0) % 360.0) - 180.0)
            if displacement > float(policy.get("xyEnvelopeM") or 0.015):
                return {"ok": False, "error": f"Trigger part moved {displacement * 1000:.1f} mm outside the cached cycle envelope."}
            if yaw_delta > float(policy.get("yawEnvelopeDeg") or 10.0):
                return {"ok": False, "error": f"Trigger part yaw changed {yaw_delta:.1f} deg outside the cached cycle envelope."}
            # Rebuild only the object-relative targets from the saved program;
            # the operator does not re-plan. Cached state seeds are copied onto
            # matching steps before the mandatory full preview.
            plan = self.scene.plan_program(program.get("steps") or [], start_angles, program.get("name") or "production", repeat_count=1)
            cached_steps = (compiled.get("planTemplate") or {}).get("steps") or []
            for step, cached_step in zip(plan.get("steps") or [], cached_steps):
                seed = cached_step.get("previewAngles") or cached_step.get("preferredJointSeedDeg")
                if seed:
                    step["preferredJointSeedDeg"] = deepcopy(seed)
        else:
            plan = deepcopy((compiled.get("planTemplate") or {}))
        if not plan.get("ok"):
            return plan
        self.service.set_end_effector(self.scene.end_effector)
        self.service.set_tool_profile(
            ((self.scene.coordinate_planner or {}).get("toolProfiles") or {}).get(self.scene.end_effector, {})
        )
        previewed = self.service.add_coordinate_preview(plan, start_angles)
        if not previewed.get("ok") or not (previewed.get("coordinatePreview") or {}).get("ok"):
            self.scene.release_plan_reservations(previewed)
            return {"ok": False, "error": (previewed.get("coordinatePreview") or {}).get("error") or previewed.get("error") or "Cycle preflight failed."}
        stale = self.scene.validate_plan_object_snapshots(previewed)
        if stale:
            self.scene.release_plan_reservations(previewed)
            return {"ok": False, "error": stale}
        return previewed

    def _execute_cycle(self, program: Dict[str, Any]) -> Dict[str, Any]:
        self._set_state("validating", activeCommand="preflight")
        plan = self._fresh_cycle_plan(program)
        if not plan.get("ok"):
            return plan
        self._set_state("running", activeCommand=(plan.get("steps") or [{}])[0].get("sourceStepId"))
        try:
            execution_plan = DashboardHandler.plan_with_speed_override(
                plan, self.session.get("speedOverridePct", 100)
            )
            result = self.service.execute_pick_plan(execution_plan, PHYSICAL_CONFIRM_TOKEN)
            result = self.scene.verify_executed_steps(result)
            self.scene.apply_executed_steps(
                result.get("executedSteps") or [], physical_run_ok=bool(result.get("ok"))
            )
            return result
        finally:
            self.scene.release_plan_reservations(plan)

    def _run(self, program: Dict[str, Any]) -> None:
        policy = program.get("runPolicy") or {}
        mode = str(policy.get("mode") or "finite")
        absent_since: Optional[float] = None
        stable = 0
        last_observation: Optional[float] = None
        waiting_for_removal = False
        try:
            while not self.stop_event.is_set():
                maximum = self.session.get("maxCycles")
                if maximum is not None and int(self.session.get("cycleCount") or 0) >= int(maximum):
                    self._set_state("completed", activeCommand=None)
                    return
                ready = mode in ("finite", "continuous")
                if mode == "external_triggered":
                    with self.lock:
                        ready = self.pending_external_trigger
                        if ready:
                            self.pending_external_trigger = False
                            self.session["pendingTrigger"] = False
                elif mode == "object_triggered":
                    if waiting_for_removal:
                        part_id = str(policy.get("triggerPartId") or "")
                        with self.scene.lock:
                            visible = part_id in self.scene.parts
                        if visible:
                            self._set_state("waiting", activeCommand=None)
                            self.stop_event.wait(0.1)
                            continue
                        absent_since = absent_since or time.monotonic()
                        if time.monotonic() - absent_since < float(policy.get("rearmAbsentMs") or 1000) / 1000.0:
                            self.stop_event.wait(0.1)
                            continue
                        waiting_for_removal = False
                        absent_since = None
                        stable = 0
                        last_observation = None
                    ready, absent_since, stable, last_observation = self._object_trigger_ready(
                        program, absent_since, stable, last_observation,
                    )
                if not ready:
                    self._set_state("waiting", activeCommand=None)
                    self.stop_event.wait(0.1)
                    continue
                result = self._execute_cycle(program)
                if not result.get("ok"):
                    self._set_state("faulted", lastError=result.get("error") or "Cycle failed.", activeCommand=None)
                    return
                with self.lock:
                    self.session["cycleCount"] = int(self.session.get("cycleCount") or 0) + 1
                    self.session["lastCompletedAt"] = time.time()
                if mode == "object_triggered":
                    waiting_for_removal = True
                cooldown = float(policy.get("cooldownMs") or 500) / 1000.0
                self._set_state("cooldown", activeCommand=None)
                self.stop_event.wait(cooldown)
        except Exception as exc:
            self._set_state("faulted", lastError=str(exc), activeCommand=None)
        finally:
            if self.stop_event.is_set():
                self._set_state("disarmed", activeCommand=None, pendingTrigger=False)


class DashboardHandler(BaseHTTPRequestHandler):
    service: RobotService
    scene: Workcell
    camera: CameraService
    localization: ContinuousLocalizationRuntime
    charuco: CharucoCalibrationSession
    production_runtime: ProductionProgramRuntime
    realtime_plans: Dict[str, Dict[str, Any]] = {}
    realtime_pending_runs: Dict[str, Dict[str, Any]] = {}
    realtime_plan_lock = threading.Lock()

    def end_headers(self) -> None:
        for name, value in SECURITY_RESPONSE_HEADERS.items():
            self.send_header(name, value)
        super().end_headers()

    def log_message(self, fmt: str, *args: Any) -> None:
        if self.path.startswith(("/api/angles", "/api/camera/tag-tracks", "/api/camera/status")):
            return
        print(f"{self.address_string()} - {fmt % args}")

    @staticmethod
    def plan_with_speed_override(plan: Dict[str, Any], speed_override_pct: Any = 100) -> Dict[str, Any]:
        """Apply a bounded physical speed reduction without trusting client-mutated steps."""
        execution_plan = deepcopy(plan)
        scale = clamp_float(speed_override_pct, 1, 100) / 100.0
        for step in execution_plan.get("steps") or []:
            if step.get("robotCommand") == "home":
                step["speed"] = max(1, round(float(step.get("speed", 15)) * scale))
            if step.get("jointTargetDeg") is not None:
                step["speed"] = max(1, round(float(step.get("speed", 20)) * scale))
            if step.get("coordsMm") is not None:
                step["coordSpeed"] = max(1, round(float(step.get("coordSpeed", 20)) * scale))
        execution_plan["speedOverridePct"] = round(scale * 100.0, 3)
        return execution_plan

    def execute_validated_plan(
        self, plan: Dict[str, Any], confirm: Any, speed_override_pct: Any = 100
    ) -> Dict[str, Any]:
        """Apply the same camera and frozen-object guard to every execution entry point."""
        reservation_plan = plan
        plan = self.plan_with_speed_override(reservation_plan, speed_override_pct)
        camera_gate_error = self.scene.physical_program_gate_error()
        if camera_gate_error:
            self.scene.release_plan_reservations(reservation_plan)
            return {"ok": False, "error": camera_gate_error}
        camera_accuracy_warning = self.scene.physical_program_warning()
        stale_target_error = self.scene.validate_plan_object_snapshots(plan)
        if stale_target_error:
            self.scene.release_plan_reservations(reservation_plan)
            return {"ok": False, "error": stale_target_error, "staleObjectPreview": True}
        self.service.set_end_effector(getattr(self.scene, "end_effector", "adaptive_gripper"))
        self.service.set_tool_profile(
            ((self.scene.coordinate_planner or {}).get("toolProfiles") or {}).get(
                getattr(self.scene, "end_effector", "adaptive_gripper"), {}
            )
        )
        try:
            result = self.service.execute_pick_plan(plan, confirm)
            if camera_accuracy_warning:
                result["cameraAccuracyWarning"] = camera_accuracy_warning
            result = self.scene.verify_executed_steps(result)
            self.scene.apply_executed_steps(
                result.get("executedSteps") or [], physical_run_ok=bool(result.get("ok"))
            )
            return result
        finally:
            self.scene.release_plan_reservations(reservation_plan)

    def do_GET(self) -> None:
        try:
            self._do_GET()
        except (BrokenPipeError, ConnectionResetError):
            return
        except ValueError as exc:
            self.write_json({"ok": False, "error": f"Invalid request: {exc}"}, status=400)
        except Exception as exc:
            self.write_json({"ok": False, "error": f"Server request failed: {exc}"}, status=500)

    def _do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/status":
            self.write_json(self.service.status())
            return
        if parsed.path == "/api/angles":
            self.write_json(self.service.get_angles())
            return
        if parsed.path == "/api/coords":
            self.write_json(self.service.get_coords())
            return
        if parsed.path == "/api/kinematics/frame-snapshot":
            self.write_json(self.service.kinematics_snapshot())
            return
        if parsed.path == "/api/ports":
            self.write_json({"ports": serial_ports()})
            return
        if parsed.path == "/api/realtime/status":
            self.write_json(realtime_status())
            return
        if parsed.path == "/api/scene":
            self.write_json(self.scene.snapshot())
            return
        if parsed.path == "/api/camera/status":
            snapshot = self.scene.snapshot()
            self.write_json({
                **self.camera.status(),
                "config": snapshot.get("camera"),
                "calibration": snapshot.get("calibration"),
            })
            return
        if parsed.path == "/api/camera/localization/status":
            self.write_json({"ok": True, **self.localization.status()})
            return
        if parsed.path == "/api/camera/tags/visible":
            self.write_json(self.localization.visible_tags())
            return
        if parsed.path == "/api/camera/tag-tracks":
            query = parse_qs(parsed.query)
            since = query.get("since", [None])[0]
            try:
                revision = int(since) if since is not None else None
            except (TypeError, ValueError):
                self.write_json({"ok": False, "error": "since must be an integer revision."}, status=400)
                return
            self.write_json(self.scene.tag_tracks(revision))
            return
        if parsed.path == "/api/program/runtime/status":
            self.write_json(self.production_runtime.status())
            return
        if parsed.path == "/api/camera/debug-frame":
            frame = self.localization.get_debug_jpeg()
            if not frame:
                self.write_json({"ok": False, "error": "No localization debug frame available."}, status=503)
            else:
                self.write_jpeg(frame)
            return
        if parsed.path == "/api/camera/devices":
            self.write_json({"ok": True, "devices": self.camera.list_devices(probe_opencv=True)})
            return
        if parsed.path == "/api/camera/frame":
            self.write_camera_frame()
            return
        if parsed.path == "/api/camera/stream":
            self.write_camera_stream()
            return

        path = parsed.path
        if path.startswith("/api/"):
            self.write_json({"ok": False, "error": f"Unknown API endpoint: {path}"}, status=404)
            return
        if path == "/":
            path = "/index.html"
        self.write_static(path)

    def do_POST(self) -> None:
        try:
            self._do_POST()
        except (BrokenPipeError, ConnectionResetError):
            return
        except ValueError as exc:
            self.write_json({"ok": False, "error": f"Invalid request: {exc}"}, status=400)
        except Exception as exc:
            self.write_json({"ok": False, "error": f"Server request failed: {exc}"}, status=500)

    def _do_POST(self) -> None:
        parsed = urlparse(self.path)
        security_error = self.post_request_security_error(parsed.path, self.headers)
        if security_error is not None:
            status, message = security_error
            self.write_json({"ok": False, "error": message}, status=status)
            return

        if parsed.path == "/api/realtime/session":
            self.create_realtime_session()
            return

        try:
            body = self.read_json()
        except ValueError as exc:
            self.write_json({"ok": False, "error": f"Invalid request body: {exc}"}, status=400)
            return

        if parsed.path == "/api/config":
            port = body.get("port")
            baud = body.get("baud")
            self.write_json(self.service.configure(port=port, baud=int(baud) if baud else None))
            return
        if parsed.path == "/api/send-angles":
            self.write_json(self.service.send_angles(body.get("angles"), body.get("speed", 20)))
            return
        if parsed.path == "/api/robot/jog/start":
            self.service.set_end_effector(getattr(self.scene, "end_effector", "adaptive_gripper"))
            self.write_json(self.service.start_joint_jog(
                body.get("jointId"), body.get("direction"), body.get("speed", 10)
            ))
            return
        if parsed.path == "/api/robot/jog/heartbeat":
            self.write_json(self.service.heartbeat_jog(body.get("sessionId")))
            return
        if parsed.path == "/api/robot/jog/step":
            self.service.set_end_effector(getattr(self.scene, "end_effector", "adaptive_gripper"))
            self.write_json(self.service.step_jog(
                body.get("space"), body.get("axisId"), body.get("increment"), body.get("speed", 10)
            ))
            return
        if parsed.path == "/api/robot/jog/stop":
            self.write_json(self.service.stop_jog())
            return
        if parsed.path.startswith("/api/command/"):
            self.service.set_end_effector(getattr(self.scene, "end_effector", "adaptive_gripper"))
            self.service.set_tool_profile(
                ((self.scene.coordinate_planner or {}).get("toolProfiles") or {}).get(
                    getattr(self.scene, "end_effector", "adaptive_gripper"), {}
                )
            )
            self.write_json(self.service.command(parsed.path.rsplit("/", 1)[-1]))
            return
        if parsed.path == "/api/realtime/tool":
            self.write_json(self.run_realtime_tool(body))
            return
        if parsed.path in ("/api/scene/part", "/api/scene/object"):
            self.write_json(self.scene.upsert_part(body))
            return
        if parsed.path == "/api/scene/part/tag-binding":
            self.write_json(self.scene.bind_tagged_part(body))
            return
        if parsed.path == "/api/scene/part/tag-unbind":
            self.write_json(self.scene.unbind_tagged_part(str(body.get("partId") or body.get("id") or "")))
            return
        if parsed.path == "/api/scene/part/delete":
            self.write_json(self.scene.delete_part(str(body.get("id") or "")))
            return
        if parsed.path == "/api/scene/bin":
            self.write_json(self.scene.upsert_bin(body))
            return
        if parsed.path == "/api/scene/bin/tag-binding":
            self.write_json(self.scene.bind_tagged_bin(body))
            return
        if parsed.path == "/api/scene/bin/tag-unbind":
            self.write_json(self.scene.unbind_tagged_bin(str(body.get("binId") or body.get("id") or "")))
            return
        if parsed.path == "/api/scene/bin/confirm-position":
            self.write_json(self.scene.confirm_bin_position(str(body.get("binId") or body.get("id") or "")))
            return
        if parsed.path == "/api/scene/bin/delete":
            self.write_json(self.scene.delete_bin(str(body.get("id") or "")))
            return
        if parsed.path == "/api/scene/support-surface":
            self.write_json(self.scene.upsert_support_surface(body))
            return
        if parsed.path == "/api/scene/support-surface/delete":
            self.write_json(self.scene.delete_support_surface(str(body.get("id") or "")))
            return
        if parsed.path == "/api/robot/points/capture":
            self.write_json(self.capture_taught_point(body))
            return
        if parsed.path == "/api/scene/point":
            self.write_json(self.scene.save_taught_point(body))
            return
        if parsed.path == "/api/scene/point/delete":
            self.write_json(self.scene.delete_taught_point(str(body.get("pointId") or body.get("id") or "")))
            return
        if parsed.path == "/api/spatial/resolve":
            self.write_json(self.scene.resolve_spatial_destination(body))
            return
        if parsed.path == "/api/scene/clear":
            self.write_json(self.scene.clear_parts())
            return
        if parsed.path == "/api/scene/end-effector":
            result = self.scene.set_end_effector(body)
            self.service.set_end_effector(result.get("endEffector"))
            self.service.set_tool_profile(
                ((result.get("coordinatePlanner") or {}).get("toolProfiles") or {}).get(
                    result.get("endEffector"), {}
                )
            )
            self.write_json(result)
            return
        if parsed.path == "/api/scene/coordinate-planner":
            result = self.scene.set_coordinate_planner_config(body)
            self.service.set_tool_profile(
                ((result.get("coordinatePlanner") or {}).get("toolProfiles") or {}).get(
                    result.get("endEffector"), {}
                )
            )
            self.write_json(result)
            return
        if parsed.path == "/api/robot/capture-tool-orientation":
            coords_result = self.service.get_coords()
            if not coords_result.get("ok") or not coords_result.get("coords"):
                self.write_json(coords_result)
                return
            coords = [float(value) for value in coords_result["coords"]]
            result = self.scene.set_coordinate_planner_config({
                "toolRpyDeg": {"rx": coords[3], "ry": coords[4], "rz": coords[5]},
                "toolRpySource": "captured",
            })
            self.write_json({
                **result,
                "capturedCoords": [round(value, 3) for value in coords],
                "capturedToolRpyDeg": {
                    "rx": round(coords[3], 3),
                    "ry": round(coords[4], 3),
                    "rz": round(coords[5], 3),
                },
            })
            return
        if parsed.path == "/api/programs/save":
            self.write_json(self.scene.save_program(body))
            return
        if parsed.path == "/api/programs/delete":
            self.write_json(self.scene.delete_program(str(body.get("id") or "")))
            return
        if parsed.path == "/api/program/plan":
            self.write_json(self.plan_program_request(body))
            return
        if parsed.path in ("/api/program/execute", "/api/pick/execute"):
            result = self.execute_validated_plan(
                body.get("plan") or {},
                body.get("confirm"),
                body.get("speedOverridePct", 100),
            )
            self.write_json(result, status=409 if result.get("staleObjectPreview") else 200)
            return
        if parsed.path == "/api/program/release-preview":
            self.scene.release_plan_reservations(body.get("plan") or {})
            self.write_json(self.scene.snapshot())
            return
        if parsed.path == "/api/program/runtime/arm":
            self.write_json(self.production_runtime.arm(
                str(body.get("programId") or ""), body.get("confirm"),
                body.get("speedOverridePct", 100),
            ))
            return
        if parsed.path == "/api/program/runtime/trigger":
            self.write_json(self.production_runtime.trigger())
            return
        if parsed.path == "/api/program/runtime/stop":
            self.write_json(self.production_runtime.stop())
            return
        if parsed.path == "/api/pick/simulate":
            self.write_json(self.scene.plan_pick(body, self.service.status()))
            return
        if parsed.path == "/api/camera/calibration":
            self.write_json(self.scene.set_calibration(body))
            return
        if parsed.path == "/api/camera/config":
            result = self.scene.set_camera_config(body)
            self.camera.configure(result.get("camera") or {})
            self.write_json(result)
            return
        if parsed.path == "/api/camera/start":
            self.camera.configure(self.scene.snapshot().get("camera") or {})
            status = self.camera.start()
            if status.get("ok") and status.get("running"):
                self.localization.start()
            else:
                self.localization.stop()
            self.write_json({"ok": bool(status.get("ok")) and bool(status.get("running")), **status})
            return
        if parsed.path == "/api/camera/stop":
            self.localization.stop()
            self.write_json(self.camera.stop())
            return
        if parsed.path == "/api/camera/calibration/charuco/clear":
            self.write_json({"ok": True, **self.charuco.clear()})
            return
        if parsed.path == "/api/camera/calibration/charuco/remove-last":
            self.write_json({"ok": True, **self.charuco.remove_last()})
            return
        if parsed.path == "/api/camera/calibration/charuco/capture":
            frame = self.camera.get_jpeg()
            self.write_json(self.charuco.capture(frame) if frame else {"ok": False, "error": "No camera frame available."})
            return
        if parsed.path == "/api/camera/calibration/charuco/solve":
            result = self.charuco.solve()
            if result.get("ok"):
                saved = self.scene.set_calibration({"intrinsics": result})
                result["calibration"] = saved.get("calibration")
            self.write_json(result)
            return
        if parsed.path == "/api/camera/calibration/workspace":
            fiducials = dict(body.get("fiducials") or {})
            saved = self.scene.set_calibration({"mode": "fiducial_table", "fiducials": fiducials})
            self.write_json(saved)
            return
        if parsed.path == "/api/camera/calibration/verify":
            result = self.localization.process_once()
            self.write_json(result)
            return
        if parsed.path == "/api/camera/calibration/verification-report":
            report = verification_report(body.get("samples") or [], body.get("stationarySpreadM"))
            saved = self.scene.set_calibration({"verification": report})
            self.write_json({"ok": True, "report": report, "calibration": saved.get("calibration")})
            return
        if parsed.path == "/api/camera/calibration/verification-skip":
            report = {
                "sampleCount": 0,
                "rmsXyErrorM": None,
                "maxXyErrorM": None,
                "stationarySpreadM": None,
                "passed": False,
                "testingBypass": True,
                "mode": "testing_unverified",
                "skippedAt": time.time(),
            }
            saved = self.scene.set_calibration({"verification": report})
            self.write_json({"ok": True, "report": report, "calibration": saved.get("calibration")})
            return
        if parsed.path == "/api/camera/calibration/accept-pose":
            intrinsics = (self.scene.snapshot().get("calibration") or {}).get("intrinsics") or {}
            if (
                not intrinsics.get("ok") or
                float(intrinsics.get("intrinsicRmsPx") or float("inf")) > 2.5 or
                float(intrinsics.get("maximumViewErrorPx") or intrinsics.get("intrinsicRmsPx") or float("inf")) > 4.0
            ):
                self.write_json({"ok": False, "error": "A passing lens calibration is required before locking the camera pose."})
                return
            result = self.localization.process_once()
            homography = result.get("homography")
            if not homography:
                self.write_json({"ok": False, "error": result.get("error") or "No valid homography."})
                return
            current = self.scene.snapshot().get("calibration") or {}
            fiducials = {**(current.get("fiducials") or {}), "baselineHomography": homography, "allowCurrentPose": False}
            saved = self.scene.set_calibration({"mode": "fiducial_table", "fiducials": fiducials})
            self.write_json({"ok": True, "quality": result.get("quality"), "calibration": saved.get("calibration")})
            return
        self.write_json({"ok": False, "error": f"Unknown API endpoint: {parsed.path}"}, status=404)

    @staticmethod
    def post_request_security_error(path: str, headers: Any) -> Optional[Tuple[int, str]]:
        """Reject cross-site or browser-simple POSTs before they reach hardware APIs."""
        fetch_site = str(headers.get("Sec-Fetch-Site") or "").strip().lower()
        if fetch_site and fetch_site not in {"same-origin", "none"}:
            return 403, "Cross-site API requests are not allowed."

        origin = str(headers.get("Origin") or "").strip()
        if origin:
            host = str(headers.get("Host") or "").strip().lower()
            parsed_origin = urlparse(origin)
            if (
                parsed_origin.scheme not in {"http", "https"}
                or not host
                or parsed_origin.netloc.lower() != host
            ):
                return 403, "Request origin does not match the dashboard."

        expected_type = (
            "application/sdp" if path == "/api/realtime/session" else "application/json"
        )
        supplied_type = str(headers.get("Content-Type") or "").split(";", 1)[0].strip().lower()
        if supplied_type != expected_type:
            return 415, f"Content-Type must be {expected_type}."
        return None

    def plan_program_request(self, body: Dict[str, Any]) -> Dict[str, Any]:
        request_started = time.perf_counter()
        steps = body.get("steps")
        name = str(body.get("name") or "ad-hoc")
        program_id = str(body.get("programId") or "")
        program: Optional[Dict[str, Any]] = None
        if not steps and body.get("programId"):
            snapshot = self.scene.snapshot()
            program = next(
                (p for p in snapshot["programs"] if p["id"] == body["programId"]), None
            )
            if program is None:
                return {"ok": False, "error": f"Program '{body['programId']}' not found."}
            steps = program["steps"]
            name = program["name"]
            repeat_count = 1 if body.get("persistCompiledCycle", True) else int(program.get("repeatCount") or 1)
        else:
            repeat_count = int(body.get("repeatCount") or 1)
        if not steps:
            return {"ok": False, "error": "No program steps were provided."}
        # A camera frame can land between clicking Validate and taking the
        # scene snapshot. Give an already outlined/stabilizing tag a bounded
        # half-second opportunity to produce a fresh pose; never wait for an
        # absent tag or extend physical execution freshness.
        for step in steps:
            if step.get("type") == "pick" and step.get("objectId"):
                self.scene.wait_for_tag_pose(str(step["objectId"]), timeout_s=0.5)
        status = self.service.status()
        start = status.get("lastAngles") or [0.0] * 6
        start_angles = [float(v) for v in start]
        expansion_started = time.perf_counter()
        plan = self.scene.plan_program(steps, start_angles, name, repeat_count=repeat_count)
        expansion_ms = (time.perf_counter() - expansion_started) * 1000.0
        compiled_seeded = False
        if program is not None and self.scene.compiled_cycle_error(program) is None:
            cached_steps = {
                str(step.get("stateId")): step
                for step in (((program.get("compiledCycle") or {}).get("planTemplate") or {}).get("steps") or [])
                if step.get("stateId")
            }
            for step in plan.get("steps") or []:
                cached = cached_steps.get(str(step.get("stateId")))
                if not cached:
                    continue
                if cached.get("previewAngles"):
                    step["preferredJointSeedDeg"] = deepcopy(cached["previewAngles"])
                    compiled_seeded = True
                if cached.get("selectedOrientation"):
                    step["preferredOrientation"] = deepcopy(cached["selectedOrientation"])
                    compiled_seeded = True
        self.service.set_end_effector(self.scene.end_effector)
        self.service.set_tool_profile(
            ((self.scene.coordinate_planner or {}).get("toolProfiles") or {}).get(
                self.scene.end_effector, {}
            )
        )
        preview_started = time.perf_counter()
        previewed = self.service.add_coordinate_preview(plan, start_angles)
        preview_ms = (time.perf_counter() - preview_started) * 1000.0
        existing_diagnostics = dict(previewed.get("planningDiagnostics") or {})
        existing_diagnostics["compiledCycleSeeded"] = compiled_seeded
        phases = {"programExpansionMs": expansion_ms, "coordinatePreviewMs": preview_ms}
        slowest_phase = max(phases, key=phases.get)
        previewed["planningDiagnostics"] = {
            **existing_diagnostics,
            **{key: round(value, 1) for key, value in phases.items()},
            "totalMs": round((time.perf_counter() - request_started) * 1000.0, 1),
            "slowestPhase": slowest_phase,
        }
        if not previewed.get("ok") or not previewed.get("coordinatePreview", {}).get("ok"):
            # A failed offline preview is not an executable plan and must not
            # leave its object frozen in a stale reservation.
            self.scene.release_plan_reservations(previewed)
        elif program_id and body.get("persistCompiledCycle", True):
            persisted = self.scene.persist_compiled_cycle(program_id, previewed, start_angles)
            if not persisted.get("ok"):
                self.scene.release_plan_reservations(previewed)
                return {**previewed, "ok": False, "error": persisted.get("error")}
            previewed["compiledCycle"] = persisted.get("compiledCycle")
            previewed["cacheStatus"] = (persisted.get("compiledCycle") or {}).get("status")
            previewed["cacheReadinessError"] = (
                persisted.get("compiledCycle") or {}
            ).get("readinessError")
        return previewed

    def capture_taught_point(self, body: Dict[str, Any]) -> Dict[str, Any]:
        # Capturing is a stationary operation.  Ending any active jog here
        # makes Save Point safe even when pointer-up was lost by the browser.
        self.service.stop_jog()
        time.sleep(0.12)
        snapshot = self.service.kinematics_snapshot()
        if not snapshot.get("ok"):
            return {"ok": False, "error": snapshot.get("error") or "Could not read the stationary robot pose."}
        angles = [float(value) for value in snapshot.get("anglesDeg") or []]
        coords = [float(value) for value in snapshot.get("firmwareFlangeCoords") or []]
        if len(angles) != 6 or len(coords) != 6 or not all(math.isfinite(value) for value in angles + coords):
            return {"ok": False, "error": "Robot returned incomplete or non-finite pose data."}
        for joint, value in enumerate(angles, 1):
            try:
                validate_joint_angle(joint, value)
            except ValueError as exc:
                return {"ok": False, "error": str(exc)}
        bounds = validate_coordinate_bounds(coords, "taught_point_capture", allow_missing_rpy=False)
        if bounds:
            return {"ok": False, "error": bounds[0].get("message") or "Captured pose is outside coordinate bounds."}
        position_error = float(snapshot.get("positionErrorMm") or 0.0)
        orientation_error = float(snapshot.get("orientationErrorDeg") or 0.0)
        if position_error > HOST_FK_HARD_POSITION_TOLERANCE_MM or orientation_error > HOST_FK_HARD_ORIENTATION_TOLERANCE_DEG:
            return {
                "ok": False,
                "error": (
                    f"Captured robot readings disagree with the host model by {position_error:.1f} mm / "
                    f"{orientation_error:.1f} deg; resolve frame calibration before saving this point."
                ),
                "kinematics": snapshot,
            }
        tool_id = self.scene.end_effector
        profile = ((self.scene.coordinate_planner or {}).get("toolProfiles") or {}).get(tool_id) or {}
        correction = profile.get("tcpCorrectionLocalM") or {}
        correction_local = [float(correction.get(axis, 0.0)) for axis in ("x", "y", "z")]
        suction_distance = float((profile.get("geometry") or {}).get("flangeToContactM", 0.072))
        flange_position = tuple(value / 1000.0 for value in coords[:3])
        flange_rotation = rotation_from_rpy_deg(coords[3:6])
        tcp_position, tcp_rotation = tcp_from_flange(
            flange_position, flange_rotation, tool_id, correction_local, suction_distance
        )
        tcp_rpy = rpy_deg_from_rotation(tcp_rotation)
        point = {
            "id": body.get("id"),
            "label": str(body.get("label") or "Taught Point"),
            "tcpPoseM": {
                "position": {"x": tcp_position[0], "y": tcp_position[1], "z": tcp_position[2]},
                "rpyDeg": {"rx": tcp_rpy[0], "ry": tcp_rpy[1], "rz": tcp_rpy[2]},
            },
            "flangePoseM": {
                "position": {"x": flange_position[0], "y": flange_position[1], "z": flange_position[2]},
                "rpyDeg": {"rx": coords[3], "ry": coords[4], "rz": coords[5]},
            },
            "firmwareFlangeCoordsMmDeg": coords,
            "jointAnglesDeg": angles,
            "endEffector": tool_id,
            "toolCalibrationFingerprint": self.scene.tool_calibration_fingerprint(tool_id),
            "supportSurfaceZ": body.get("supportSurfaceZ"),
            "uses": body.get("uses") or ["waypoint", "destination"],
            "capturedAt": time.time(),
        }
        if body.get("persist"):
            return self.scene.save_taught_point(point)
        return {"ok": True, "pointDraft": point, "kinematics": snapshot}

    def _resolve_visible_part(self, arguments: Dict[str, Any]) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
        snapshot = self.scene.snapshot()
        parts = snapshot.get("parts") or []
        part_id = str(arguments.get("objectId") or arguments.get("entityId") or "")
        if part_id:
            match = next((part for part in parts if str(part.get("id")) == part_id), None)
            if match:
                return match, None
            hidden = next(
                (item for item in snapshot.get("registeredParts") or [] if str(item.get("partId")) == part_id),
                None,
            )
            if hidden:
                return None, f"{hidden.get('label') or part_id} is registered, but its AprilTag is not currently visible. Restore the camera/tag view before planning."
            return None, f"Visible part '{part_id}' was not found."
        query = str(arguments.get("objectQuery") or arguments.get("label") or "").strip().lower()
        if not query:
            return None, "Specify the visible part by ID or name."
        exact = [part for part in parts if str(part.get("label") or "").strip().lower() == query]
        if len(exact) == 1:
            return exact[0], None
        partial = [part for part in parts if query in str(part.get("label") or "").lower()]
        if len(partial) == 1:
            return partial[0], None
        if len(exact) > 1 or len(partial) > 1:
            return None, f"'{query}' matches more than one visible part; use the exact part ID."
        hidden_matches = [
            item for item in snapshot.get("registeredParts") or []
            if query == str(item.get("label") or "").strip().lower()
            or query in str(item.get("label") or "").lower()
        ]
        if len(hidden_matches) == 1 and not hidden_matches[0].get("visible"):
            hidden = hidden_matches[0]
            return None, f"{hidden.get('label') or hidden.get('partId')} is registered, but its AprilTag is not currently visible. Restore the camera/tag view before planning."
        return None, f"No visible part matches '{query}'."

    def plan_spatial_move(self, arguments: Dict[str, Any]) -> Dict[str, Any]:
        part, error = self._resolve_visible_part(arguments)
        if part is None:
            return {"ok": False, "error": error}
        request = {
            "entityKind": "part",
            "entityId": part["id"],
            "destination": arguments.get("destination") or {
                "kind": arguments.get("destinationKind") or "region",
                "region": arguments.get("region") or "center",
                "pointId": arguments.get("pointId"),
                "binId": arguments.get("binId"),
                "dxM": arguments.get("dxM"),
                "dyM": arguments.get("dyM"),
                "referenceKind": arguments.get("referenceKind"),
                "referenceId": arguments.get("referenceId"),
                "side": arguments.get("side"),
            },
        }
        resolved = self.scene.resolve_spatial_destination(request)
        if not resolved.get("ok"):
            return resolved
        name = str(arguments.get("name") or f"Move {part.get('label')} {resolved.get('destinationKind')}")
        failures = []
        # Candidates are ordered by minimum travel from the object's current
        # pose. Avoid turning one voice request into dozens of expensive full
        # firmware/host IK previews when a region is crowded or unreachable.
        for candidate in (resolved.get("candidates") or [])[:6]:
            if resolved.get("destinationKind") == "bin":
                place = {"type": "place", "binId": candidate.get("binId")}
            elif resolved.get("destinationKind") == "point":
                place = {"type": "place", "pointId": candidate.get("pointId")}
            else:
                place = {"type": "place", "position": candidate.get("position")}
            program_steps = [
                {"type": "pick", "objectId": part["id"]},
                place,
            ]
            plan = self.plan_program_request({"name": name, "steps": program_steps})
            if plan.get("ok") and (plan.get("coordinatePreview") or {}).get("ok"):
                plan.setdefault("notes", []).append(str(resolved.get("coordinateReason") or "Spatial destination resolved by the server."))
                plan["spatialResolution"] = {
                    "destinationKind": resolved.get("destinationKind"),
                    "region": resolved.get("region"),
                    "selectedPosition": deepcopy(candidate.get("position")),
                    "coordinateReason": resolved.get("coordinateReason"),
                }
                saved = self.scene.save_program({"name": name, "steps": program_steps})
                program = saved.get("program") or {"name": name, "steps": program_steps}
                return {
                    "ok": True,
                    "program": program,
                    "plan": self.remember_realtime_plan(plan, "plan_spatial_move"),
                    "spatialResolution": plan["spatialResolution"],
                }
            failures.append({
                "position": candidate.get("position"),
                "error": (plan.get("coordinatePreview") or {}).get("error") or plan.get("error"),
            })
        return {
            "ok": False,
            "error": "No collision-free spatial candidate passed complete-path IK validation.",
            "spatialResolution": resolved,
            "candidateFailures": failures,
        }

    def update_virtual_layout(self, arguments: Dict[str, Any]) -> Dict[str, Any]:
        entity_kind = str(arguments.get("entityKind") or "bin")
        if entity_kind != "bin":
            return {"ok": False, "error": "Only bins can be repositioned by the AI layout tool."}
        bin_id = str(arguments.get("binId") or arguments.get("entityId") or "")
        resolved = self.scene.resolve_spatial_destination({
            "entityKind": "bin", "entityId": bin_id,
            "destination": arguments.get("destination") or {
                "kind": arguments.get("destinationKind") or "region",
                "region": arguments.get("region") or "center",
                "dxM": arguments.get("dxM"), "dyM": arguments.get("dyM"),
                "pointId": arguments.get("pointId"),
            },
        })
        if not resolved.get("ok"):
            return resolved
        candidate = (resolved.get("candidates") or [None])[0]
        if not candidate:
            return {"ok": False, "error": "No virtual layout position was found."}
        snapshot = self.scene.snapshot()
        bin_obj = next((item for item in snapshot.get("bins") or [] if item.get("id") == bin_id), None)
        if bin_obj is None:
            return {"ok": False, "error": "Bin was not found."}
        updated = self.scene.upsert_bin({
            **bin_obj,
            "position": candidate["position"],
            "positionStatus": "simulation_only",
            "positionSource": "ai_spatial_layout",
        })
        return {
            "ok": True,
            "bin": updated.get("bin"),
            "simulationOnly": True,
            "physicalRunBlocked": True,
            "warning": "The virtual bin moved; the physical bin did not. Confirm its real position before a physical run.",
            "spatialResolution": resolved,
        }

    def plan_move_to_point(self, arguments: Dict[str, Any]) -> Dict[str, Any]:
        snapshot = self.scene.snapshot()
        point_id = str(arguments.get("pointId") or "")
        if not point_id and arguments.get("name"):
            query = str(arguments["name"]).strip().lower()
            matches = [point for point in snapshot.get("taughtPoints") or [] if str(point.get("label") or "").strip().lower() == query]
            if len(matches) == 1:
                point_id = str(matches[0]["id"])
        point = next((item for item in snapshot.get("taughtPoints") or [] if item.get("id") == point_id), None)
        if point is None:
            return {"ok": False, "error": "Taught point was not found or was ambiguous."}
        program_steps = [{"type": "move_to_point", "pointId": point_id}]
        name = str(arguments.get("programName") or f"Go to {point['label']}")
        plan = self.plan_program_request({"name": name, "steps": program_steps})
        if not plan.get("ok") or not (plan.get("coordinatePreview") or {}).get("ok"):
            return plan
        saved = self.scene.save_program({"name": name, "steps": program_steps})
        return {
            "ok": True,
            "program": saved.get("program"),
            "plan": self.remember_realtime_plan(plan, "plan_move_to_point"),
        }

    @classmethod
    def prune_realtime_plans(cls) -> None:
        now = time.time()
        expired = [
            plan_id for plan_id, record in cls.realtime_plans.items()
            if now - float(record.get("createdAt") or 0.0) > REALTIME_PLAN_TTL_S
        ]
        for plan_id in expired:
            cls.realtime_plans.pop(plan_id, None)
        expired_runs = [
            run_id for run_id, record in cls.realtime_pending_runs.items()
            if now - float(record.get("createdAt") or 0.0) > REALTIME_RUN_CONFIRM_TTL_S
        ]
        for run_id in expired_runs:
            cls.realtime_pending_runs.pop(run_id, None)

    @classmethod
    def realtime_plan_payload(cls, plan: Dict[str, Any]) -> Dict[str, Any]:
        payload = json.loads(json.dumps(plan))
        safety = payload.get("safetyGate")
        if isinstance(safety, dict):
            safety.pop("physicalConfirmToken", None)
            safety["voiceRunFlow"] = "Say 'run it'; then answer yes or no once."
            safety["reason"] = (
                "Physical execution requires one explicit yes/no confirmation after the run request."
            )
        return payload

    @classmethod
    def remember_realtime_plan(cls, plan: Dict[str, Any], source: str) -> Dict[str, Any]:
        if not plan.get("ok"):
            return plan
        plan_id = f"rtplan-{uuid.uuid4().hex[:12]}"
        created_at = time.time()
        with cls.realtime_plan_lock:
            cls.prune_realtime_plans()
            cls.realtime_plans[plan_id] = {
                "plan": plan,
                "source": source,
                "createdAt": created_at,
            }
        return {
            **cls.realtime_plan_payload(plan),
            "realtimePlanId": plan_id,
            "expiresInSeconds": REALTIME_PLAN_TTL_S,
            "executionGate": {
                "requiresPlanId": True,
                "requiresVerbalYes": True,
                "note": "Say 'run it', then answer the single yes/no confirmation.",
            },
        }

    def execute_realtime_plan(self, arguments: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "ok": False,
            "confirmationRequired": True,
            "error": (
                "This legacy direct-execution tool is disabled. Use request_program_run first, "
                "then confirm_program_run with its pendingRunId after the user says yes."
            ),
        }

    def save_realtime_program(self, arguments: Dict[str, Any]) -> Dict[str, Any]:
        result = self.scene.save_program({
            "id": arguments.get("id"),
            "name": arguments.get("name") or "Voice Program",
            "editorVersion": 2,
            "repeatCount": arguments.get("repeatCount", 1),
            "steps": arguments.get("steps") or [],
        })
        if not result.get("ok"):
            return result
        program = result.get("program") or {}
        plan = self.plan_program_request({"programId": program.get("id")})
        return {
            "ok": True,
            "program": program,
            "plan": self.remember_realtime_plan(plan, "save_program"),
            "sceneVersion": result.get("version"),
        }

    def save_pick_place_program(self, arguments: Dict[str, Any]) -> Dict[str, Any]:
        with self.scene.lock:
            part = self.scene.parts.get(str(arguments.get("objectId") or ""))
            if part is None:
                part = self.scene.match_part(str(arguments.get("objectQuery") or arguments.get("prompt") or "part"))
            bin_obj = self.scene.bins.get(str(arguments.get("binId") or arguments.get("destinationId") or ""))
            if bin_obj is None:
                bin_obj = next(iter(self.scene.bins.values()), None)
        if part is None:
            return {"ok": False, "error": "No matching part found to save a pick/place program."}
        if bin_obj is None:
            return {"ok": False, "error": "No bin found to save a pick/place program."}
        name = str(arguments.get("name") or f"pick_{part['label']}_to_{bin_obj['label']}")
        safe_id = "".join(ch.lower() if ch.isalnum() else "-" for ch in name).strip("-") or "voice-pick-place"
        return self.save_realtime_program({
            "id": arguments.get("id") or safe_id,
            "name": name,
            "steps": [
                {"type": "home"},
                {"type": "pick", "objectId": part["id"]},
                {"type": "place", "binId": bin_obj["id"]},
                {"type": "home"},
            ],
        })

    def execute_saved_program(self, arguments: Dict[str, Any]) -> Dict[str, Any]:
        # Backward-compatible alias that can only stage a run.  A yes or an
        # internal token in this first call is deliberately ignored.
        return self.request_voice_program_run(arguments)

    def resolve_saved_program(self, arguments: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        snapshot = self.scene.snapshot()
        program_id = str(arguments.get("programId") or "")
        program_name = str(arguments.get("name") or "").strip().lower()
        if program_id:
            match = next((p for p in snapshot["programs"] if p.get("id") == program_id), None)
            if match is not None:
                return match
        if program_name:
            return next(
                (p for p in snapshot["programs"] if str(p.get("name") or "").strip().lower() == program_name),
                None,
            )
        if len(snapshot["programs"]) == 1:
            return snapshot["programs"][0]
        return None

    def request_voice_program_run(self, arguments: Dict[str, Any]) -> Dict[str, Any]:
        realtime_plan_id = str(arguments.get("realtimePlanId") or "")
        with self.realtime_plan_lock:
            self.prune_realtime_plans()
            if not realtime_plan_id and not any(
                arguments.get(key) for key in ("programId", "name")
            ) and self.realtime_plans:
                # "Run that" means the most recently previewed plan. This is
                # especially important for Home, which is intentionally a
                # temporary preview rather than a saved dashboard program.
                realtime_plan_id = max(
                    self.realtime_plans,
                    key=lambda plan_id: float(self.realtime_plans[plan_id].get("createdAt") or 0.0),
                )
            realtime_record = self.realtime_plans.get(realtime_plan_id) if realtime_plan_id else None
        if realtime_plan_id:
            if realtime_record is None:
                return {"ok": False, "error": "That preview expired. Plan it again, then say run it."}
            plan = realtime_record["plan"]
            saved_program = self.resolve_saved_program({"name": plan.get("program")})
            if saved_program is not None and realtime_record.get("source") != "plan_home_zero":
                program = {**saved_program, "realtimePlanId": realtime_plan_id}
            else:
                program = {
                    "id": None,
                    "name": str(plan.get("program") or "Current preview"),
                    "steps": ([{"type": "home"}] if realtime_record.get("source") == "plan_home_zero" else []),
                    "temporaryPreview": True,
                    "realtimePlanId": realtime_plan_id,
                }
        else:
            program = self.resolve_saved_program(arguments)
            if program is None:
                return {"ok": False, "error": "Program not found. Preview it again, then say run it."}
            plan = self.plan_program_request({"programId": program["id"]})
            if not plan.get("ok"):
                return plan
        run_id = f"rtrun-{uuid.uuid4().hex[:12]}"
        with self.realtime_plan_lock:
            self.prune_realtime_plans()
            self.realtime_pending_runs[run_id] = {
                "program": program,
                "plan": plan,
                "createdAt": time.time(),
            }
        return {
            "ok": True,
            "pendingRunId": run_id,
            "expiresInSeconds": REALTIME_RUN_CONFIRM_TTL_S,
            "program": program,
            "plan": self.realtime_plan_payload(plan),
            "confirmationRequired": True,
            "confirmationPrompt": f"Run {program['name']} now?",
        }

    def confirm_voice_program_run(self, arguments: Dict[str, Any]) -> Dict[str, Any]:
        answer = str(arguments.get("answer") or "").strip().lower()
        run_id = str(arguments.get("pendingRunId") or "")
        if not run_id:
            return {"ok": False, "error": "pendingRunId is required; request the program run again first."}
        with self.realtime_plan_lock:
            self.prune_realtime_plans()
            record = self.realtime_pending_runs.pop(run_id, None)
        if record is None:
            return {"ok": False, "error": "No pending voice run found. Say run on the robot again first."}
        if answer not in {"yes", "y", "confirm", "confirmed", "run", "go"}:
            self.scene.release_plan_reservations(record["plan"])
            return {"ok": False, "cancelled": True, "error": "Physical run cancelled."}
        result = self.execute_validated_plan(record["plan"], PHYSICAL_CONFIRM_TOKEN)
        return {"program": record["program"], **result}

    def read_body(self) -> bytes:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return b""
        if length > MAX_REQUEST_BODY_BYTES:
            raise ValueError(f"Request body exceeds {MAX_REQUEST_BODY_BYTES} bytes")
        return self.rfile.read(length)

    def read_json(self) -> Dict[str, Any]:
        body = self.read_body()
        if not body:
            return {}
        def reject_non_finite(value: str) -> None:
            raise ValueError(f"non-finite JSON number {value!r} is not allowed")

        payload = json.loads(body.decode("utf-8"), parse_constant=reject_non_finite)
        if not isinstance(payload, dict):
            raise ValueError("JSON request body must be an object")
        return payload

    def write_json(self, payload: Dict[str, Any], status: int = 200) -> None:
        data = json.dumps(json_safe(payload), allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def write_camera_frame(self) -> None:
        frame = self.camera.get_jpeg()
        if not frame:
            self.write_json({"ok": False, "error": "No camera frame available."}, status=503)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.end_headers()
        self.wfile.write(frame)

    def write_jpeg(self, frame: bytes) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.end_headers()
        self.wfile.write(frame)

    def write_camera_stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.end_headers()
        try:
            while True:
                frame = self.camera.get_jpeg()
                if frame:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                elif not self.camera.is_running():
                    return
                time.sleep(0.12)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def write_static(self, request_path: str) -> None:
        try:
            file_path = (STATIC_ROOT / request_path.lstrip("/")).resolve()
        except (OSError, ValueError):
            self.send_error(400)
            return
        # Containment check that also covers symlinks (Path.is_relative_to needs 3.9+).
        if not str(file_path).startswith(str(STATIC_ROOT) + os.sep):
            self.send_error(400)
            return
        if not file_path.is_file():
            self.send_error(404)
            return

        content = file_path.read_bytes()
        content_type = mimetypes.guess_type(file_path.name)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        self.wfile.write(content)

    def write_text(self, text: str, content_type: str = "text/plain", status: int = 200) -> None:
        data = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def create_realtime_session(self) -> None:
        api_key = os.environ.get("OPENAI_API_KEY")
        if not api_key:
            self.write_text("OPENAI_API_KEY is not set on the Python server.", status=500)
            return

        try:
            sdp = self.read_body().decode("utf-8")
            answer = create_openai_realtime_call(api_key, sdp)
            self.write_text(answer, content_type="application/sdp")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            self.write_text(f"OpenAI Realtime HTTP {exc.code}: {detail}", status=502)
        except Exception as exc:
            self.write_text(f"OpenAI Realtime session failed: {exc}", status=502)

    def run_realtime_tool(self, body: Dict[str, Any]) -> Dict[str, Any]:
        name = body.get("name")
        arguments = body.get("arguments") or {}

        try:
            if name in ("get_robot_status", "get_robot_angles"):
                return self.service.get_angles()
            if name == "send_robot_angles":
                return {"ok": False, "error": "Voice control cannot send arbitrary joint angles."}
            if name == "stop_robot":
                return self.service.command("stop")
            if name == "plan_home_zero":
                return self.remember_realtime_plan(
                    self.plan_program_request({"name": "voice home", "steps": [{"type": "home"}]}),
                    "plan_home_zero",
                )
            if name == "open_gripper":
                return self.service.command("gripper-open")
            if name == "close_gripper":
                return self.service.command("gripper-close")
            if name == "get_environment_scene":
                return {
                    "ok": True,
                    "scene": self.scene.assistant_snapshot(),
                    "spatialContext": self.scene.spatial_context(),
                    "robot": self.service.status(),
                }
            if name == "get_spatial_context":
                return {**self.scene.spatial_context(), "robot": self.service.status()}
            if name == "list_taught_points":
                snapshot = self.scene.snapshot()
                return {"ok": True, "taughtPoints": snapshot.get("taughtPoints") or []}
            if name == "plan_spatial_move":
                return self.plan_spatial_move(arguments)
            if name == "update_virtual_layout":
                return self.update_virtual_layout(arguments)
            if name == "plan_move_to_point":
                return self.plan_move_to_point(arguments)
            if name == "classify_visible_part":
                part_id = str(arguments.get("partId") or "")
                with self.scene.lock:
                    part = deepcopy(self.scene.parts.get(part_id))
                if not part or part.get("trackingMode") != "apriltag" or not self.scene.tag_pose_is_fresh(part):
                    return {"ok": False, "error": "That registered part is not currently visible to the camera."}
                frame = self.camera.get_jpeg()
                if not frame:
                    return {"ok": False, "error": "No current camera frame is available."}
                return object_classifier.classify_visible_part(frame, part)
            if name == "apply_part_identity":
                part_id = str(arguments.get("partId") or "")
                if arguments.get("confirmed") is not True:
                    return {"ok": False, "error": "Explicit user confirmation is required before changing the part identity."}
                with self.scene.lock:
                    if part_id not in self.scene.registered_parts:
                        return {"ok": False, "error": "That registered part was not found."}
                shape = str(arguments.get("shape") or "unknown")
                if shape not in {"box", "cylinder", "sphere", "rectangle", "circle", "unknown"}:
                    return {"ok": False, "error": "Unsupported part shape."}
                return self.scene.upsert_part({"id": part_id, "label": arguments.get("label"), "type": shape})
            if name in ("plan_pick_place", "simulate_pick_object"):
                return self.save_pick_place_program(arguments)
            if name == "plan_program":
                if arguments.get("name") and arguments.get("steps"):
                    return self.save_realtime_program(arguments)
                return self.remember_realtime_plan(self.plan_program_request(arguments), "plan_program")
            if name in ("save_program", "save_current_program", "click_save_button", "save_program_to_dashboard"):
                return self.save_realtime_program(arguments)
            if name == "request_program_run":
                return self.request_voice_program_run(arguments)
            if name == "confirm_program_run":
                return self.confirm_voice_program_run(arguments)
            if name in ("execute_planned_program", "execute_confirmed_plan"):
                return self.execute_realtime_plan(arguments)
            if name == "execute_saved_program":
                return self.execute_saved_program(arguments)
            return {"ok": False, "error": f"Unknown Realtime tool: {name}"}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}


def serial_ports() -> Any:
    try:
        from serial.tools import list_ports
    except Exception:
        return []
    return [
        {"device": port.device, "description": port.description or "n/a"}
        for port in list_ports.comports()
    ]


def realtime_status() -> Dict[str, Any]:
    return {
        "configured": bool(os.environ.get("OPENAI_API_KEY")),
        "model": REALTIME_MODEL,
        "voice": REALTIME_VOICE,
    }


def clamp_float(value: Any, minimum: float, maximum: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = minimum
    return max(minimum, min(maximum, number))


def create_openai_realtime_call(api_key: str, sdp: str) -> str:
    boundary = f"----mycobot-realtime-{uuid.uuid4().hex}"
    session = {
        "type": "realtime",
        "model": REALTIME_MODEL,
        "reasoning": {"effort": "low"},
        "max_output_tokens": 160,
        "output_modalities": ["audio"],
        "audio": {
            "input": {"turn_detection": None},
            "output": {"voice": REALTIME_VOICE},
        },
        "instructions": (
            "# Role and objective\n"
            "You are the spatial planning assistant for a local myCobot 280 M5 workcell. Be extremely concise. "
            "Default to one short sentence. NEVER speak before calling a tool. Forbidden filler includes 'Okay', 'Alright', 'Got it', 'I will', and 'let me'. "
            "Never narrate tool calls, restate the request, recap steps, or offer extra options unless asked.\n"
            "# Spatial grounding\n"
            "Before any spatial command, call get_spatial_context. Robot frame: +X front, -X back, +Y left, -Y right, +Z up. "
            "Use exact IDs returned by tools. Read availabilityWarnings, but mention only a warning that blocks the requested action. "
            "Never calculate, infer, or invent XYZ coordinates yourself. Hidden tagged parts have no usable location. "
            "Do not call get_spatial_context for home, run confirmation, stop, or robot-status requests.\n"
            "# Planning\n"
            "For requests such as move a part right, next to something, to a taught point, or into a bin, call plan_spatial_move. "
            "For go-to-point requests call plan_move_to_point. Tool results create the dashboard program and simulation. "
            "Call planning tools silently with no preceding audio. Wait for each result before speaking or trying another destination. On success say exactly 'Plan ready.' "
            "On failure state only the specific blocker. Never claim a tool is still running after it returned. "
            "Named-region and relative moves create a transient server-calculated destination. Persistent taught points can only be created from measured robot capture, never from model reasoning. "
            "A simulation_only bin is not a verified physical destination. update_virtual_layout moves only the digital bin and must be described that way.\n"
            "# Safety\n"
            "Do not move the physical robot unless the user explicitly requests the current plan to run. Preserve the realtimePlanId returned by every planning tool. "
            "For 'run that' or equivalent, call request_program_run with that realtimePlanId with no preceding audio. Ask exactly the returned short confirmation question once. "
            "After a clear yes, call confirm_program_run immediately with no preceding audio, then say exactly 'Run complete.' or the exact failure. Never ask for a program name when a realtimePlanId exists. "
            "Never request arbitrary joint angles or reveal internal confirmation tokens. "
            "Stop is always allowed. Gripper open/close is allowed only when explicitly requested.\n"
            "# Classification\n"
            "Only classify a currently visible tagged part when explicitly requested. Present the suggestion before apply_part_identity; classification never changes geometry."
        ),
        "tools": [
            {
                "type": "function",
                "name": "get_robot_status",
                "description": "Read current robot status, including joint angles, connection state, errors, and execution state.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
            {
                "type": "function",
                "name": "stop_robot",
                "description": "Immediately send the robot stop command.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
            {
                "type": "function",
                "name": "plan_home_zero",
                "description": "Plan a conservative move to the configured home/zero posture. This only simulates and returns a realtimePlanId; it does not move the robot.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
            {
                "type": "function",
                "name": "open_gripper",
                "description": "Open the adaptive gripper.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
            {
                "type": "function",
                "name": "close_gripper",
                "description": "Close the adaptive gripper.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
            {
                "type": "function",
                "name": "get_environment_scene",
                "description": "Read visible spatial parts and bins plus the registered tagged inventory. Hidden registered parts have no current coordinates.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
            {
                "type": "function",
                "name": "get_spatial_context",
                "description": "Read calibrated workspace regions, visible parts, bins, taught points, and derived spatial relationships before planning any spatial request.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
            {
                "type": "function",
                "name": "list_taught_points",
                "description": "List named measured robot points and whether each can be used as a waypoint or object destination.",
                "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
            },
            {
                "type": "function",
                "name": "plan_spatial_move",
                "description": "Create and simulate a validated pick/place plan to a named region, bin, taught point, relative offset, or position next to another scene object. Server code chooses coordinates; never supply invented XYZ values.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "objectId": {"type": "string"},
                        "objectQuery": {"type": "string"},
                        "destinationKind": {"type": "string", "enum": ["region", "bin", "point", "relative", "next_to"]},
                        "region": {"type": "string", "enum": ["left", "right", "front", "back", "center"]},
                        "binId": {"type": "string"},
                        "pointId": {"type": "string"},
                        "dxM": {"type": "number"},
                        "dyM": {"type": "number"},
                        "referenceKind": {"type": "string", "enum": ["part", "bin"]},
                        "referenceId": {"type": "string"},
                        "side": {"type": "string", "enum": ["left", "right", "front", "back"]}
                    },
                    "additionalProperties": False,
                },
            },
            {
                "type": "function",
                "name": "update_virtual_layout",
                "description": "Move a bin only in the digital simulation. The server marks it unverified and blocks physical use until the operator confirms the real bin position.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "binId": {"type": "string"},
                        "destinationKind": {"type": "string", "enum": ["region", "point", "relative"]},
                        "region": {"type": "string", "enum": ["left", "right", "front", "back", "center"]},
                        "pointId": {"type": "string"},
                        "dxM": {"type": "number"},
                        "dyM": {"type": "number"}
                    },
                    "required": ["binId", "destinationKind"],
                    "additionalProperties": False,
                },
            },
            {
                "type": "function",
                "name": "plan_move_to_point",
                "description": "Create and simulate a validated robot move to an existing taught point. Does not physically move until the normal run confirmation flow.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "pointId": {"type": "string"},
                        "name": {"type": "string"},
                        "programName": {"type": "string"}
                    },
                    "additionalProperties": False,
                },
            },
            {
                "type": "function", "name": "classify_visible_part",
                "description": "On explicit user request, analyze one currently visible tagged part and return a name/shape suggestion without changing it.",
                "parameters": {"type": "object", "properties": {"partId": {"type": "string"}}, "required": ["partId"], "additionalProperties": False},
            },
            {
                "type": "function", "name": "apply_part_identity",
                "description": "After explicit user confirmation, apply a suggested label and shape to a registered part. Never changes pose or dimensions.",
                "parameters": {"type": "object", "properties": {"partId": {"type": "string"}, "label": {"type": "string"}, "shape": {"type": "string", "enum": ["box", "cylinder", "sphere", "rectangle", "circle", "unknown"]}, "confirmed": {"type": "boolean", "description": "True only after the user explicitly confirms this exact suggestion."}}, "required": ["partId", "label", "shape", "confirmed"], "additionalProperties": False},
            },
            {
                "type": "function",
                "name": "plan_pick_place",
                "description": "Create and save a dashboard program that picks a scene part and places it in a bin, then returns a plan preview. Does not move the physical robot until request_program_run and confirm_program_run are used.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "id": {"type": "string"},
                        "name": {"type": "string"},
                        "objectQuery": {
                            "type": "string",
                            "description": "Natural target phrase such as 'red block'.",
                        },
                        "objectId": {"type": "string"},
                        "binId": {"type": "string"},
                    },
                    "additionalProperties": False,
                },
            },
            {
                "type": "function",
                "name": "plan_program",
                "description": "Create a named sequential dashboard program using visible parts, bins, and existing taught point IDs. May add motion, pick/place, tool, home, and wait nodes, but must never invent coordinates or embedded waypoints.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string"},
                        "steps": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "type": {"type": "string", "enum": ["pick", "place", "home", "move", "move_to_point", "tool", "acquire", "release", "wait"]},
                                    "label": {"type": "string"},
                                    "enabled": {"type": "boolean"},
                                    "motionType": {"type": "string", "enum": ["joint", "linear"]},
                                    "speed": {"type": "integer", "minimum": 1, "maximum": 100},
                                    "action": {"type": "string", "enum": ["acquire", "release"]},
                                    "durationMs": {"type": "integer", "minimum": 50, "maximum": 600000},
                                    "objectId": {"type": "string"},
                                    "binId": {"type": "string"},
                                    "pointId": {"type": "string"},
                                },
                                "required": ["type"],
                                "additionalProperties": False,
                            },
                        },
                        "repeatCount": {"type": "integer", "minimum": 1, "maximum": 20},
                    },
                    "required": ["steps"],
                    "additionalProperties": False,
                },
            },
            {
                "type": "function",
                "name": "save_program",
                "description": "Create or update a saved teach-pendant program. Motion nodes may reference only existing pointId values. Also returns a simulated plan preview and never moves the robot.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "id": {"type": "string", "description": "Optional existing program id to update."},
                        "name": {"type": "string"},
                        "steps": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "type": {"type": "string", "enum": ["pick", "place", "home", "move", "move_to_point", "tool", "acquire", "release", "wait"]},
                                    "label": {"type": "string"},
                                    "enabled": {"type": "boolean"},
                                    "motionType": {"type": "string", "enum": ["joint", "linear"]},
                                    "speed": {"type": "integer", "minimum": 1, "maximum": 100},
                                    "action": {"type": "string", "enum": ["acquire", "release"]},
                                    "durationMs": {"type": "integer", "minimum": 50, "maximum": 600000},
                                    "objectId": {"type": "string"},
                                    "binId": {"type": "string"},
                                    "pointId": {"type": "string"},
                                },
                                "required": ["type"],
                                "additionalProperties": False,
                            },
                        },
                        "repeatCount": {"type": "integer", "minimum": 1, "maximum": 20},
                    },
                    "required": ["name", "steps"],
                    "additionalProperties": False,
                },
            },
            {
                "type": "function",
                "name": "request_program_run",
                "description": "Stage the current preview or a saved dashboard program for physical execution and return a pendingRunId. Prefer the realtimePlanId from the most recent planning result. This does not move the robot; ask one short yes/no question before confirming.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "realtimePlanId": {"type": "string"},
                        "programId": {"type": "string"},
                        "name": {"type": "string"},
                    },
                    "additionalProperties": False,
                },
            },
            {
                "type": "function",
                "name": "confirm_program_run",
                "description": "Execute a previously staged physical robot run after the user verbally confirms yes.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "pendingRunId": {"type": "string"},
                        "answer": {"type": "string", "enum": ["yes", "no"]},
                    },
                    "required": ["pendingRunId", "answer"],
                    "additionalProperties": False,
                },
            },
        ],
        "tool_choice": "auto",
    }

    body = multipart_body(
        boundary,
        [
            ("sdp", "application/sdp", sdp),
            ("session", "application/json", json.dumps(session)),
        ],
    )
    request = urllib.request.Request(
        "https://api.openai.com/v1/realtime/calls",
        data=body,
        method="POST",
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": f"multipart/form-data; boundary={boundary}",
            "OpenAI-Safety-Identifier": "local-mycobot-dashboard",
        },
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return response.read().decode("utf-8")


def multipart_body(boundary: str, fields: Any) -> bytes:
    chunks = []
    for name, content_type, value in fields:
        chunks.append(f"--{boundary}\r\n".encode("utf-8"))
        chunks.append(f'Content-Disposition: form-data; name="{name}"\r\n'.encode("utf-8"))
        chunks.append(f"Content-Type: {content_type}\r\n\r\n".encode("utf-8"))
        chunks.append(value.encode("utf-8"))
        chunks.append(b"\r\n")
    chunks.append(f"--{boundary}--\r\n".encode("utf-8"))
    return b"".join(chunks)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the myCobot digital twin web dashboard.")
    parser.add_argument("--port", help="Robot serial port, for example /dev/cu.usbserial-XXXXXXXX")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.8)
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Loopback bind address only (127.0.0.1 or localhost).",
    )
    parser.add_argument("--web-port", type=int, default=8765)
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.list:
        return list_serial_ports()
    if not is_loopback_bind_host(args.host):
        print(
            "Refusing non-loopback HTTP binding. CobotWorkcell exposes physical-control "
            "endpoints and does not provide remote authentication."
        )
        print("Use --host 127.0.0.1 and access the dashboard only from this computer.")
        return 2

    DashboardHandler.service = RobotService(args.port, args.baud, args.timeout)
    DashboardHandler.scene = Workcell(ROOT / "data")
    DashboardHandler.camera = CameraService(DashboardHandler.scene.camera)
    DashboardHandler.charuco = CharucoCalibrationSession()
    DashboardHandler.localization = ContinuousLocalizationRuntime(DashboardHandler.camera, DashboardHandler.scene)
    DashboardHandler.production_runtime = ProductionProgramRuntime(
        DashboardHandler.scene, DashboardHandler.service
    )
    DashboardHandler.service.set_end_effector(DashboardHandler.scene.end_effector)
    DashboardHandler.service.set_tool_profile(
        ((DashboardHandler.scene.coordinate_planner or {}).get("toolProfiles") or {}).get(
            DashboardHandler.scene.end_effector, {}
        )
    )
    try:
        server = ThreadingHTTPServer((args.host, args.web_port), DashboardHandler)
    except OSError as exc:
        print(f"Cannot listen on {args.host}:{args.web_port} - {exc.strerror or exc}.")
        print("Another web_server.py is probably still running. Find it with:")
        print(f"    lsof -nP -iTCP:{args.web_port} -sTCP:LISTEN")
        print("then kill that PID, or start this server with a different --web-port.")
        return 1
    print(f"Dashboard: http://{args.host}:{args.web_port}")
    if args.port:
        print(f"Robot serial: {args.port} @ {args.baud}")
    else:
        print("Robot serial: choose a port in the webpage")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard.")
    finally:
        DashboardHandler.production_runtime.shutdown()
        DashboardHandler.localization.stop()
        DashboardHandler.camera.stop()
        DashboardHandler.service.configure(port=None)
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
