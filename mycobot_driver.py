#!/usr/bin/env python3
"""
pymycobot-backed transport for the myCobot 280 M5.

Wraps Elephant Robotics' official ``pymycobot`` library (robust framing,
internal retries, blocking sync moves) behind the small method surface the web
server already used for the hand-rolled UART driver, plus the extra primitives
needed for the firmware linear-descent path: ``sync_send_angles``,
``get_coords``, ``send_coords``, ``is_moving`` and ``is_in_position``.

Why this exists: the previous raw-UART driver dropped the serial port on a
single transient read (a timeout or one short/corrupt frame), which is the root
cause of the mid-run "stalls and disconnects". pymycobot handles framing and
retries internally, and here every read that returns no usable data is turned
into ``RobotReadError`` so the caller's existing miss-tolerance logic keeps
working instead of killing the connection.

Install dependency:
    python3 -m pip install pymycobot
"""

from __future__ import annotations

import os
import math
import time
from typing import Any, Dict, List, Optional, Sequence

try:
    from pymycobot import MyCobot280
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit(
        "Missing dependency: pymycobot\n"
        "Install it with: python3 -m pip install pymycobot"
    ) from exc


class RobotReadError(RuntimeError):
    """A read (angles / coords / gripper) returned no usable data.

    Raised instead of propagating pymycobot's -1 / [] / None sentinels, so the
    server treats it as a transient miss (retry) rather than a fatal error.
    """


def _valid_sextuple(value) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) >= 6
        and all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in value[:6])
    )


class MyCobotDriver:
    """Adapter exposing the method names ``web_server.py`` calls."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.8) -> None:
        self.port = port
        self.baud = int(baud)
        self.timeout = float(timeout)
        # Surfaced for the existing diagnostics in web_server (read via getattr).
        self.last_discarded_frames = 0
        self.last_drained_bytes = 0
        self._mc = MyCobot280(port, str(self.baud), timeout=self.timeout, debug=False)
        # Let the board settle and flush boot chatter before the first command.
        time.sleep(0.2)

    # ----------------------------------------------------------- lifecycle
    def close(self) -> None:
        try:
            self._mc.close()
        except Exception:
            port = getattr(self._mc, "_serial_port", None)
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass

    def drain_input(self) -> int:
        """Clear stale UART bytes; mirrors the old driver's diagnostic hook."""
        port = getattr(self._mc, "_serial_port", None)
        if port is None:
            self.last_drained_bytes = 0
            return 0
        try:
            pending = int(getattr(port, "in_waiting", 0) or 0)
            port.reset_input_buffer()
            self.last_drained_bytes = pending
            return pending
        except Exception:
            self.last_drained_bytes = 0
            return 0

    def __enter__(self) -> "MyCobotDriver":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # --------------------------------------------------------------- reads
    def get_angles(self, response_timeout: Optional[float] = None) -> List[float]:
        result = self._mc.get_angles()
        if not _valid_sextuple(result):
            raise RobotReadError(f"get_angles returned no usable data: {result!r}")
        return [float(v) for v in result[:6]]

    def get_coords(self, response_timeout: Optional[float] = None) -> List[float]:
        result = self._mc.get_coords()
        if not _valid_sextuple(result):
            raise RobotReadError(f"get_coords returned no usable data: {result!r}")
        return [float(v) for v in result[:6]]

    def is_moving(self) -> Optional[int]:
        result = self._mc.is_moving()
        return int(result) if isinstance(result, (int, float)) and result >= 0 else None

    def is_in_position(self, data: Sequence[float], id: int = 0) -> Optional[int]:
        try:
            result = self._mc.is_in_position(list(data), id)
        except Exception:
            return None
        return int(result) if isinstance(result, (int, float)) and result >= 0 else None

    def is_power_on(self, response_timeout: Optional[float] = None) -> Optional[int]:
        result = self._mc.is_power_on()
        return int(result) if isinstance(result, (int, float)) else None

    def get_error_information(self) -> Optional[int]:
        """Return the controller's current motion/IK error code, if available."""
        if not hasattr(self._mc, "get_error_information"):
            return None
        result = self._mc.get_error_information()
        if isinstance(result, (list, tuple)) and result:
            result = result[0]
        return int(result) if isinstance(result, (int, float)) and result >= 0 else None

    # -------------------------------------------------------------- motion
    def send_angles(self, degrees: Sequence[float], speed: int) -> None:
        self._mc.send_angles([float(v) for v in degrees], int(speed))

    def jog_angle(self, joint_id: int, direction: int, speed: int) -> None:
        self._mc.jog_angle(int(joint_id), int(direction), int(speed))

    def jog_increment_angle(self, joint_id: int, increment: float, speed: int) -> None:
        self._mc.jog_increment_angle(int(joint_id), float(increment), int(speed))

    def jog_coord(self, coord_id: int, direction: int, speed: int) -> None:
        self._mc.jog_coord(int(coord_id), int(direction), int(speed))

    def jog_increment_coord(self, coord_id: int, increment: float, speed: int) -> None:
        self._mc.jog_increment_coord(int(coord_id), float(increment), int(speed))

    def jog_stop(self) -> None:
        if hasattr(self._mc, "jog_stop"):
            self._mc.jog_stop()
        else:  # pragma: no cover - compatibility with older pymycobot builds
            self._mc.stop()

    def sync_send_angles(
        self, degrees: Sequence[float], speed: int, timeout: float = 15.0
    ) -> None:
        self._mc.sync_send_angles([float(v) for v in degrees], int(speed), timeout)

    def send_coords(self, coords: Sequence[float], speed: int, mode: int = 1) -> None:
        self._mc.send_coords([float(v) for v in coords], int(speed), int(mode))

    def sync_send_coords(
        self, coords: Sequence[float], speed: int, mode: int = 1, timeout: float = 15.0
    ) -> None:
        self._mc.sync_send_coords([float(v) for v in coords], int(speed), int(mode), timeout)

    def solve_inv_kinematics(
        self,
        target_coords: Sequence[float],
        current_angles: Sequence[float],
    ) -> List[float]:
        if not hasattr(self._mc, "solve_inv_kinematics"):
            raise RuntimeError("This pymycobot driver does not expose solve_inv_kinematics.")
        result = self._mc.solve_inv_kinematics(
            [float(v) for v in target_coords],
            [float(v) for v in current_angles],
        )
        if not _valid_sextuple(result):
            raise RobotReadError(f"solve_inv_kinematics returned no usable data: {result!r}")
        return [float(v) for v in result[:6]]

    def angles_to_coords(self, angles: Sequence[float]) -> List[float]:
        """Read-only firmware FK for validating an IK result without motion."""
        if not hasattr(self._mc, "angles_to_coords"):
            raise RuntimeError("This pymycobot driver does not expose angles_to_coords.")
        result = self._mc.angles_to_coords([float(v) for v in angles])
        if not _valid_sextuple(result):
            raise RobotReadError(f"angles_to_coords returned no usable data: {result!r}")
        return [float(v) for v in result[:6]]

    def stop(self) -> None:
        self._mc.stop()

    # --------------------------------------------------------- servo power
    def power_on(self) -> None:
        self._mc.power_on()

    def power_off(self) -> None:
        self._mc.power_off()

    def focus_all_servos(self) -> None:
        # pymycobot exposes per-servo focus; power_on re-enables torque on all.
        self._mc.power_on()

    def release_all_servos(self) -> None:
        self._mc.release_all_servos()

    # ------------------------------------------------------------- gripper
    def set_gripper_state(self, flag: int, speed: int) -> None:
        self._mc.set_gripper_state(int(flag), int(speed))

    def open_gripper(self, speed: int = 60) -> None:
        self._mc.set_gripper_state(0, int(speed))

    def close_gripper(self, speed: int = 60) -> None:
        self._mc.set_gripper_state(1, int(speed))

    def auto_grip(self, speed: int = 35) -> None:
        self._mc.set_gripper_state(1, int(speed))

    def release_gripper(self, speed: int = 60) -> None:
        # The adaptive gripper has no torque-release flag in pymycobot; open it.
        self._mc.set_gripper_state(0, int(speed))

    def set_gripper_value(self, value: int, speed: int) -> None:
        self._mc.set_gripper_value(int(value), int(speed))

    @staticmethod
    def _suction_profile_sequences(profile: str) -> tuple[str, str]:
        profiles = {
            # Pump 2.0 default: pin 5 pump, pin 2 release valve, both
            # active-low. Close the valve before starting the pump. To release,
            # stop the pump and vent for one full second.
            "pump_v2": ("2:1,5:0", "5:1,2:0,sleep:1.0,2:1"),
            # Named legacy profile for the older/reversed harness.
            "legacy_split_valve": ("2:0,5:1", "2:1,5:0,sleep:0.35,5:1"),
            # Older examples wire both outputs active-low together.
            "both_low": ("2:0,5:0", "2:1,5:1"),
            # Useful if the pump/valve harness is reversed.
            "inverted_split": ("2:1,5:0", "2:0,5:1,sleep:0.35,5:0"),
        }
        return profiles.get(profile, profiles["pump_v2"])

    @staticmethod
    def _parse_suction_sequence(value: str) -> List[Dict[str, Any]]:
        sequence: List[Dict[str, Any]] = []
        for item in str(value or "").split(","):
            token = item.strip()
            if not token:
                continue
            lower = token.lower()
            if lower.startswith("sleep:") or lower.startswith("delay:"):
                _, seconds = token.split(":", 1)
                sequence.append({"type": "sleep", "seconds": max(0.0, float(seconds.strip()))})
                continue
            parts = token.split(":")
            if len(parts) not in (2, 3):
                raise ValueError(f"Invalid suction output token: {token!r}")
            pin = int(parts[0].strip())
            signal = int(parts[1].strip())
            if signal not in (0, 1):
                raise ValueError(f"Suction output signal must be 0 or 1: {token!r}")
            action: Dict[str, Any] = {"type": "basic_output", "pin": pin, "signal": signal}
            if len(parts) == 3:
                action["delayAfter"] = max(0.0, float(parts[2].strip()))
            sequence.append(action)
        return sequence

    def set_suction(self, enabled: bool) -> Dict[str, Any]:
        """Control the M5 Basic suction pump outputs.

        This accessory is plugged into the M5 base, so it must use
        set_basic_output rather than the Atom/tool-head digital outputs.

        Pump 2.0 defaults use output 5 as the pump and output 2 as the release valve:
        suction on => pump active, valve closed; suction off => pump off, vent
        briefly, valve closed. Override with MYCOBOT_SUCTION_ON /
        MYCOBOT_SUCTION_OFF as comma-separated actions such as:
        "2:1,5:0" or "5:1,2:0,sleep:1.0,2:1".
        """
        profile = os.environ.get("MYCOBOT_SUCTION_PROFILE", "pump_v2").strip() or "pump_v2"
        default_on, default_off = self._suction_profile_sequences(profile)
        default = default_on if enabled else default_off
        env_key = "MYCOBOT_SUCTION_ON" if enabled else "MYCOBOT_SUCTION_OFF"
        sequence = self._parse_suction_sequence(os.environ.get(env_key, default))
        if not hasattr(self._mc, "set_basic_output"):
            raise RuntimeError("This pymycobot driver does not expose set_basic_output for suction control.")
        for action in sequence:
            if action["type"] == "sleep":
                time.sleep(float(action["seconds"]))
                continue
            self._mc.set_basic_output(int(action["pin"]), int(action["signal"]))
            time.sleep(float(action.get("delayAfter", 0.05)))
        return {
            "enabled": bool(enabled),
            "profile": profile,
            "sequence": sequence,
        }

    def set_suction_output(self, pin: int, signal: int) -> Dict[str, Any]:
        if not hasattr(self._mc, "set_basic_output"):
            raise RuntimeError("This pymycobot driver does not expose set_basic_output for suction control.")
        pin_value = int(pin)
        signal_value = int(signal)
        if signal_value not in (0, 1):
            raise ValueError("Suction diagnostic signal must be 0 or 1.")
        self._mc.set_basic_output(pin_value, signal_value)
        time.sleep(0.05)
        return {
            "enabled": None,
            "profile": os.environ.get("MYCOBOT_SUCTION_PROFILE", "pump_v2").strip() or "pump_v2",
            "sequence": [{"type": "basic_output", "pin": pin_value, "signal": signal_value}],
            "diagnostic": True,
        }

    def suction_on(self) -> Dict[str, Any]:
        return self.set_suction(True)

    def suction_off(self) -> Dict[str, Any]:
        return self.set_suction(False)

    def get_gripper_value(self, response_timeout: Optional[float] = None) -> Optional[int]:
        result = self._mc.get_gripper_value()
        return int(result) if isinstance(result, (int, float)) and result >= 0 else None

    def is_gripper_moving(self, response_timeout: Optional[float] = None) -> Optional[int]:
        result = self._mc.is_gripper_moving()
        return int(result) if isinstance(result, (int, float)) and result >= 0 else None

    def set_color(self, red: int, green: int, blue: int) -> None:
        self._mc.set_color(int(red), int(green), int(blue))
