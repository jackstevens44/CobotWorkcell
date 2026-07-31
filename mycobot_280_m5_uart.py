#!/usr/bin/env python3
"""
Talk to an Elephant Robotics myCobot 280 M5 over USB/UART with raw protocol
frames.

Install dependency:
    python3 -m pip install pyserial

Common ports:
    macOS:  /dev/cu.SLAB_USBtoUART, /dev/cu.usbserial-*
    Linux:  /dev/ttyUSB0 or /dev/ttyACM0
    Windows: COM3, COM4, ...

Examples:
    python3 mycobot_280_m5_uart.py list
    python3 mycobot_280_m5_uart.py --port /dev/cu.SLAB_USBtoUART get-angles
    python3 mycobot_280_m5_uart.py --port /dev/cu.SLAB_USBtoUART send-angles 0 0 0 0 0 0 --speed 30
    python3 mycobot_280_m5_uart.py --port /dev/cu.SLAB_USBtoUART send-angle 1 20 --speed 25
    python3 mycobot_280_m5_uart.py --port /dev/cu.SLAB_USBtoUART stop
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: pyserial\n"
        "Install it with: python3 -m pip install pyserial"
    ) from exc


HEADER = bytes([0xFE, 0xFE])
FOOTER = 0xFA


class Command:
    POWER_ON = 0x10
    POWER_OFF = 0x11
    IS_POWER_ON = 0x12
    RELEASE_ALL_SERVOS = 0x13
    FOCUS_ALL_SERVOS = 0x18
    GET_ANGLES = 0x20
    SEND_ANGLE = 0x21
    SEND_ANGLES = 0x22
    STOP = 0x29
    GET_GRIPPER_VALUE = 0x65
    SET_GRIPPER_STATE = 0x66
    SET_GRIPPER_VALUE = 0x67
    SET_GRIPPER_CALIBRATION = 0x68
    IS_GRIPPER_MOVING = 0x69
    SET_COLOR = 0x6A


JOINT_LIMITS = {
    1: (-168.0, 168.0),
    2: (-135.0, 135.0),
    3: (-150.0, 150.0),
    4: (-145.0, 145.0),
    5: (-155.0, 160.0),
    6: (-180.0, 180.0),
}


@dataclass
class MyCobotUART:
    port: str
    baud: int = 115200
    timeout: float = 0.3
    write_delay: float = 0.05

    def __post_init__(self) -> None:
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        self.last_discarded_frames = 0
        self.last_drained_bytes = 0
        time.sleep(0.2)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self) -> None:
        self.serial.close()

    def __enter__(self) -> "MyCobotUART":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def frame(self, command: int, payload: Iterable[int] = ()) -> bytes:
        data = [command, *payload]
        length = len(data) + 1
        return HEADER + bytes([length, *data, FOOTER])

    def send_frame(
        self,
        command: int,
        payload: Iterable[int] = (),
        expected_command: Optional[int] = None,
        response_timeout: Optional[float] = None,
    ) -> Optional[bytes]:
        self.last_discarded_frames = 0
        self.last_drained_bytes = 0
        if expected_command is not None:
            self.drain_input()
        packet = self.frame(command, payload)
        self.serial.write(packet)
        self.serial.flush()
        time.sleep(self.write_delay)

        if expected_command is None:
            return None
        return self.read_response(expected_command, response_timeout)

    def drain_input(self) -> int:
        """Clear stale UART bytes before commands that expect a response."""
        try:
            pending = int(getattr(self.serial, "in_waiting", 0) or 0)
            self.serial.reset_input_buffer()
            self.last_drained_bytes = pending
            return pending
        except Exception:
            self.last_drained_bytes = 0
            return 0

    def read_response(
        self, expected_command: int, response_timeout: Optional[float] = None
    ) -> bytes:
        deadline = time.monotonic() + (response_timeout or self.timeout)
        buffer = bytearray()
        self.last_discarded_frames = 0

        while time.monotonic() < deadline:
            chunk = self.serial.read(1)
            if not chunk:
                continue

            buffer.extend(chunk)
            if len(buffer) >= 2 and buffer[-2:] == HEADER:
                buffer = bytearray(HEADER)

            if len(buffer) >= 5:
                length = buffer[2]
                full_size = 2 + 1 + length
                if len(buffer) >= full_size:
                    frame = bytes(buffer[:full_size])
                    if frame[-1] != FOOTER:
                        buffer = bytearray()
                        continue
                    if frame[3] != expected_command:
                        self.last_discarded_frames += 1
                        buffer = bytearray()
                        continue
                    return frame

        raise TimeoutError(f"No response for command 0x{expected_command:02X}")

    def power_on(self) -> None:
        self.send_frame(Command.POWER_ON)

    def power_off(self) -> None:
        self.send_frame(Command.POWER_OFF)

    def release_all_servos(self) -> None:
        self.send_frame(Command.RELEASE_ALL_SERVOS)

    def focus_all_servos(self) -> None:
        self.send_frame(Command.FOCUS_ALL_SERVOS)

    def stop(self) -> None:
        self.send_frame(Command.STOP)

    def set_gripper_state(self, flag: int, speed: int) -> None:
        if flag not in (0, 1, 254):
            raise ValueError("gripper flag must be 0=open, 1=close, or 254=release")
        validate_speed(speed)
        self.send_frame(Command.SET_GRIPPER_STATE, [flag, speed])

    def open_gripper(self, speed: int = 60) -> None:
        self.set_gripper_state(0, speed)

    def close_gripper(self, speed: int = 60) -> None:
        self.set_gripper_state(1, speed)

    def auto_grip(self, speed: int = 35) -> None:
        self.set_gripper_state(1, speed)

    def release_gripper(self, speed: int = 60) -> None:
        self.set_gripper_state(254, speed)

    def get_gripper_value(self, response_timeout: Optional[float] = None) -> Optional[int]:
        response = self.send_frame(
            Command.GET_GRIPPER_VALUE,
            expected_command=Command.GET_GRIPPER_VALUE,
            response_timeout=response_timeout,
        )
        payload = self.response_payload(response)
        return payload[0] if payload else None

    def is_gripper_moving(self, response_timeout: Optional[float] = None) -> Optional[int]:
        response = self.send_frame(
            Command.IS_GRIPPER_MOVING,
            expected_command=Command.IS_GRIPPER_MOVING,
            response_timeout=response_timeout,
        )
        payload = self.response_payload(response)
        return payload[0] if payload else None

    def set_gripper_value(self, value: int, speed: int) -> None:
        if not 0 <= value <= 100:
            raise ValueError("gripper value must be 0-100")
        validate_speed(speed)
        self.send_frame(Command.SET_GRIPPER_VALUE, [value, speed])

    def is_power_on(self, response_timeout: Optional[float] = None) -> Optional[int]:
        response = self.send_frame(
            Command.IS_POWER_ON,
            expected_command=Command.IS_POWER_ON,
            response_timeout=response_timeout,
        )
        payload = self.response_payload(response)
        return payload[0] if payload else None

    def get_angles(self, response_timeout: Optional[float] = None) -> List[float]:
        response = self.send_frame(
            Command.GET_ANGLES,
            expected_command=Command.GET_ANGLES,
            response_timeout=response_timeout,
        )
        payload = self.response_payload(response)
        if len(payload) < 12:
            raise ValueError(f"Short angle response payload: {payload.hex(' ')}")
        return [decode_signed_word(payload[i], payload[i + 1]) / 100.0 for i in range(0, 12, 2)]

    def send_angle(self, joint: int, degrees: float, speed: int) -> None:
        validate_joint_angle(joint, degrees)
        validate_speed(speed)
        high, low = encode_signed_word(degrees, scale=100)
        self.send_frame(Command.SEND_ANGLE, [joint, high, low, speed])

    def send_angles(self, degrees: Iterable[float], speed: int) -> None:
        values = list(degrees)
        if len(values) != 6:
            raise ValueError("send_angles requires exactly 6 joint angles")
        for joint, degrees_value in enumerate(values, start=1):
            validate_joint_angle(joint, degrees_value)
        validate_speed(speed)

        payload: List[int] = []
        for degrees_value in values:
            payload.extend(encode_signed_word(degrees_value, scale=100))
        payload.append(speed)
        self.send_frame(Command.SEND_ANGLES, payload)

    def set_color(self, red: int, green: int, blue: int) -> None:
        for name, value in {"red": red, "green": green, "blue": blue}.items():
            if not 0 <= value <= 255:
                raise ValueError(f"{name} must be 0-255")
        self.send_frame(Command.SET_COLOR, [red, green, blue])

    def raw(self, command: int, payload: Iterable[int], read: bool) -> Optional[bytes]:
        return self.send_frame(command, payload, expected_command=command if read else None)

    @staticmethod
    def response_payload(frame: Optional[bytes]) -> bytes:
        if frame is None:
            return b""
        return frame[4:-1]


def encode_signed_word(value: float, scale: int = 1) -> List[int]:
    raw = int(round(value * scale))
    if not -32768 <= raw <= 32767:
        raise ValueError(f"Scaled value {raw} does not fit in signed 16-bit range")
    raw &= 0xFFFF
    return [(raw >> 8) & 0xFF, raw & 0xFF]


def decode_signed_word(high: int, low: int) -> int:
    raw = (high << 8) | low
    return raw - 0x10000 if raw > 0x7FFF else raw


def validate_joint_angle(joint: int, degrees: float) -> None:
    if joint not in JOINT_LIMITS:
        raise ValueError("joint must be 1-6")
    low, high = JOINT_LIMITS[joint]
    if not low <= degrees <= high:
        raise ValueError(f"joint {joint} angle must be between {low} and {high} degrees")


def validate_speed(speed: int) -> None:
    if not 1 <= speed <= 100:
        raise ValueError("speed must be 1-100")


def parse_hex_byte(value: str) -> int:
    cleaned = value.strip().lower()
    if cleaned.startswith("0x"):
        cleaned = cleaned[2:]
    byte = int(cleaned, 16)
    if not 0 <= byte <= 255:
        raise argparse.ArgumentTypeError(f"{value!r} is not a byte")
    return byte


def list_serial_ports() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1
    for port in ports:
        print(f"{port.device}\t{port.description}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control a myCobot 280 M5 over USB/UART with raw serial frames."
    )
    parser.add_argument("--port", help="Serial port, for example /dev/cu.SLAB_USBtoUART or COM3")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate, default 115200")
    parser.add_argument("--timeout", type=float, default=0.5, help="Serial response timeout in seconds")
    parser.add_argument("--verbose", action="store_true", help="Print raw response frames when available")

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list", help="List serial ports")
    subparsers.add_parser("power-on", help="Power on / lock servos")
    subparsers.add_parser("power-off", help="Power off")
    subparsers.add_parser("release", help="Release all servos")
    subparsers.add_parser("focus-all", help="Lock/focus all servos")
    subparsers.add_parser("stop", help="Stop current motion")
    subparsers.add_parser("gripper-open", help="Open the adaptive gripper")
    subparsers.add_parser("gripper-auto", help="Auto grip with the adaptive gripper")
    subparsers.add_parser("gripper-close", help="Close the adaptive gripper")
    subparsers.add_parser("gripper-release", help="Release the adaptive gripper")
    subparsers.add_parser("get-gripper-value", help="Read adaptive gripper value")
    subparsers.add_parser("is-gripper-moving", help="Read adaptive gripper moving state")
    subparsers.add_parser("is-power-on", help="Read power state")
    subparsers.add_parser("get-angles", help="Read all six joint angles")

    send_angle = subparsers.add_parser("send-angle", help="Move one joint")
    send_angle.add_argument("joint", type=int, choices=range(1, 7))
    send_angle.add_argument("degrees", type=float)
    send_angle.add_argument("--speed", type=int, default=30)

    send_angles = subparsers.add_parser("send-angles", help="Move all six joints")
    send_angles.add_argument("degrees", nargs=6, type=float)
    send_angles.add_argument("--speed", type=int, default=30)

    color = subparsers.add_parser("set-color", help="Set Atom LED color")
    color.add_argument("red", type=int)
    color.add_argument("green", type=int)
    color.add_argument("blue", type=int)

    gripper_value = subparsers.add_parser("gripper-value", help="Move adaptive gripper to 0-100")
    gripper_value.add_argument("value", type=int)
    gripper_value.add_argument("--speed", type=int, default=60)

    raw = subparsers.add_parser("raw", help="Send a raw command byte and optional payload bytes")
    raw.add_argument("command_byte", type=parse_hex_byte)
    raw.add_argument("payload", nargs="*", type=parse_hex_byte)
    raw.add_argument("--read", action="store_true", help="Wait for a response with the same command byte")

    return parser


def require_port(args: argparse.Namespace) -> str:
    if not args.port:
        raise SystemExit("A serial port is required. Run 'list' first or pass --port PORT.")
    return args.port


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.command == "list":
        return list_serial_ports()

    try:
        with MyCobotUART(require_port(args), baud=args.baud, timeout=args.timeout) as robot:
            response: Optional[bytes] = None

            if args.command == "power-on":
                robot.power_on()
                print("Sent power-on.")
            elif args.command == "power-off":
                robot.power_off()
                print("Sent power-off.")
            elif args.command == "release":
                robot.release_all_servos()
                print("Released all servos.")
            elif args.command == "focus-all":
                robot.focus_all_servos()
                print("Focused all servos.")
            elif args.command == "stop":
                robot.stop()
                print("Sent stop.")
            elif args.command == "gripper-open":
                robot.open_gripper()
                print("Opened gripper.")
            elif args.command == "gripper-auto":
                robot.auto_grip()
                print("Auto grip sent.")
            elif args.command == "gripper-close":
                robot.close_gripper()
                print("Closed gripper.")
            elif args.command == "gripper-release":
                robot.release_gripper()
                print("Released gripper.")
            elif args.command == "get-gripper-value":
                print(f"Gripper value: {robot.get_gripper_value()}")
            elif args.command == "is-gripper-moving":
                print(f"Gripper moving: {robot.is_gripper_moving()}")
            elif args.command == "is-power-on":
                state = robot.is_power_on()
                print(f"Power state: {state}")
            elif args.command == "get-angles":
                angles = robot.get_angles()
                print("Angles:", " ".join(f"{angle:.2f}" for angle in angles))
            elif args.command == "send-angle":
                robot.send_angle(args.joint, args.degrees, args.speed)
                print(f"Sent joint {args.joint} to {args.degrees:.2f} degrees at speed {args.speed}.")
            elif args.command == "send-angles":
                robot.send_angles(args.degrees, args.speed)
                formatted = " ".join(f"{angle:.2f}" for angle in args.degrees)
                print(f"Sent angles [{formatted}] at speed {args.speed}.")
            elif args.command == "set-color":
                robot.set_color(args.red, args.green, args.blue)
                print(f"Set color to RGB({args.red}, {args.green}, {args.blue}).")
            elif args.command == "gripper-value":
                robot.set_gripper_value(args.value, args.speed)
                print(f"Sent gripper value {args.value} at speed {args.speed}.")
            elif args.command == "raw":
                response = robot.raw(args.command_byte, args.payload, args.read)
                sent_payload = " ".join(f"{byte:02X}" for byte in args.payload)
                print(f"Sent raw command 0x{args.command_byte:02X} payload [{sent_payload}].")
                if response:
                    print(f"Response: {response.hex(' ').upper()}")

            if args.verbose and response:
                print(f"Raw response: {response.hex(' ').upper()}")
            return 0
    except (serial.SerialException, TimeoutError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
