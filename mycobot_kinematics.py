#!/usr/bin/env python3
"""
Forward and inverse kinematics for the myCobot 280 M5 and installed tools.

The frame chain mirrors static/app's createOfficialMyCobot() exactly (same URDF
translations, rotations, and physical-correction quaternions), so the server,
the browser digital twin, and the physical robot all agree on one model.

Conventions:
- Angles in/out of the public API are physical joint degrees as reported by
  get_angles() / accepted by send_angles().
- Positions are meters in the robot base frame: +X forward, +Y left, +Z up.
- The tool (TCP) is the grasp pocket between the open gripper fingers.
- "Top-down" grasp: tool approach axis points straight down (-Z); unreachable
  vertical poses are rejected instead of silently becoming side approaches.

Pure Python on purpose: no numpy, runs on the stock python3 (3.8+).
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]

IDENTITY: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

# Constant translation from the visual/URDF base used by _CHAIN to the
# controller's firmware coordinate frame. Measured from a stationary physical
# pose plus two firmware IK solutions; spread was below 0.6 mm and orientation
# agreed within 0.03 degrees. Keep the raw model available for the digital twin
# and apply this only when comparing with firmware flange coordinates.
FIRMWARE_BASE_TRANSLATION_M: Vec3 = (0.00182, 0.00130, 0.00762)

JOINT_LIMITS_DEG: Dict[int, Tuple[float, float]] = {
    1: (-168.0, 168.0),
    2: (-135.0, 135.0),
    3: (-150.0, 150.0),
    4: (-145.0, 145.0),
    5: (-155.0, 160.0),
    6: (-180.0, 180.0),
}

# Grasp pocket depth inside the open claws, from the gripper group origin.
GRIPPER_POCKET_M = 0.078
# The adaptive-gripper CAD's mirrored distal fingertip surfaces have their
# midpoint 4 mm along gripper-local +Z from the group origin. This is part of
# the tool frame, not a camera/workspace bias.
GRIPPER_JAW_LATERAL_M = 0.004

IK_POSITION_TOLERANCE_M = 0.0005
IK_ORIENTATION_TOLERANCE_RAD = math.radians(1.0)
IK_MAX_ITERATIONS = 140
IK_DAMPING = 0.06
IK_MAX_STEP_RAD = math.radians(9.0)
IK_VALIDATION_POSITION_TOLERANCE_M = 0.003
IK_VALIDATION_ORIENTATION_TOLERANCE_RAD = math.radians(3.0)
# Picking prefers an exact top-down pose. The planner may use a small, fixed
# outward tilt to recover reach, but never changes orientation during a
# descend/lift segment.
APPROACH_TILTS_DEG = (0.0, 2.0, 4.0, 6.0, 8.0, 10.0)
MAX_TOP_DOWN_TILT_DEG = 10.0


def _rotx(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _roty(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rotz(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _mat_mul(a: Mat3, b: Mat3) -> Mat3:
    (a00, a01, a02), (a10, a11, a12), (a20, a21, a22) = a
    (b00, b01, b02), (b10, b11, b12), (b20, b21, b22) = b
    return (
        (a00 * b00 + a01 * b10 + a02 * b20,
         a00 * b01 + a01 * b11 + a02 * b21,
         a00 * b02 + a01 * b12 + a02 * b22),
        (a10 * b00 + a11 * b10 + a12 * b20,
         a10 * b01 + a11 * b11 + a12 * b21,
         a10 * b02 + a11 * b12 + a12 * b22),
        (a20 * b00 + a21 * b10 + a22 * b20,
         a20 * b01 + a21 * b11 + a22 * b21,
         a20 * b02 + a21 * b12 + a22 * b22),
    )


def _mat_vec(m: Mat3, v: Sequence[float]) -> Vec3:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _mat_t(m: Mat3) -> Mat3:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def _euler_xyz(rx: float, ry: float, rz: float) -> Mat3:
    # three.js Euler order "XYZ": R = Rx * Ry * Rz (matches the frontend model).
    return _mat_mul(_rotx(rx), _mat_mul(_roty(ry), _rotz(rz)))


def _vec_add(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_norm(a: Sequence[float]) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _vec_scale(a: Sequence[float], s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _vec_cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _vec_unit(a: Sequence[float]) -> Vec3:
    n = _vec_norm(a)
    if n < 1e-12:
        return (0.0, 0.0, 1.0)
    return _vec_scale(a, 1.0 / n)


# Joint frame chain, child-of-previous. Each entry:
#   (translation, fixed rotation R0, pre-correction C, display offset degrees)
# Local transform = Trans(t) . C . R0 . Rz(theta_deg + offset).
_HALF_PI = math.pi / 2.0
_CHAIN = (
    ((0.0, 0.0, 0.0), IDENTITY, IDENTITY, 0.0),
    ((0.0, 0.0, 0.13156), _euler_xyz(0.0, _HALF_PI, -_HALF_PI), _rotz(-_HALF_PI), 90.0),
    ((-0.1104, 0.0, 0.0), IDENTITY, IDENTITY, 0.0),
    ((-0.096, 0.0, 0.06462), _euler_xyz(0.0, 0.0, -_HALF_PI), IDENTITY, 0.0),
    ((0.0, -0.07318, 0.0), _euler_xyz(_HALF_PI, -_HALF_PI, 0.0), _rotz(_HALF_PI), 90.0),
    ((0.0, 0.0456, 0.0), _euler_xyz(-_HALF_PI, 0.0, 0.0), IDENTITY, 0.0),
)
# Precomputed static rotation per frame (C . R0) and offsets in radians, so the
# FK hot loop does one matrix multiply per joint.
_CHAIN_FAST = tuple(
    (translation, _mat_mul(correction, base_rot), math.radians(offset_deg))
    for translation, base_rot, correction, offset_deg in _CHAIN
)

# Flange -> active tool. Corrections are expressed in the tool/TCP frame so
# they rotate with the wrist. This prevents a physical left/high correction
# from becoming a false camera or part-coordinate offset.
ADAPTIVE_TOOL_ROTATION: Mat3 = _euler_xyz(_HALF_PI, math.pi / 4.0, 0.0)
_TOOL_TRANSLATION: Vec3 = (0.0, 0.01, 0.035)
_TOOL_ROTATION: Mat3 = ADAPTIVE_TOOL_ROTATION
_TOOL_POCKET: Vec3 = (0.0, GRIPPER_POCKET_M, GRIPPER_JAW_LATERAL_M)
ADAPTIVE_TOOL_TCP_VECTOR: Vec3 = _vec_add(
    _TOOL_TRANSLATION,
    _mat_vec(ADAPTIVE_TOOL_ROTATION, _TOOL_POCKET),
)
_TOOL_TCP_VECTOR: Vec3 = ADAPTIVE_TOOL_TCP_VECTOR
# The official pump-head mount rotates its local +Y axis onto the flange axis.
# The installed head is then clocked 90 degrees counterclockwise when viewed
# along the outward J6 face normal (the requested 180-degree correction from
# the prior rear-view interpretation). This does not change the cup's grasp yaw,
# but it does rotate tool-local calibration corrections with the real mount.
# The installed measurement is authoritative for contact: 50 mm rigid head to
# cup start + 22 mm free cup extension = 72 mm flange-to-contact.
SUCTION_FACE_CLOCKING_RAD = math.pi / 2.0
SUCTION_TOOL_ROTATION: Mat3 = _mat_mul(
    _rotz(SUCTION_FACE_CLOCKING_RAD), _rotx(1.579)
)
SUCTION_CONTACT_DISTANCE_M = 0.072
SUCTION_MOUNT_TRANSLATION: Vec3 = (0.0, 0.0, 0.010)
SUCTION_TOOL_TCP_VECTOR: Vec3 = _vec_add(
    SUCTION_MOUNT_TRANSLATION,
    _mat_vec(
        SUCTION_TOOL_ROTATION,
        (0.0, SUCTION_CONTACT_DISTANCE_M - SUCTION_MOUNT_TRANSLATION[2], 0.0),
    ),
)


def tool_transform(
    tool_id: str = "adaptive_gripper",
    correction_local_m: Optional[Sequence[float]] = None,
    suction_contact_distance_m: float = SUCTION_CONTACT_DISTANCE_M,
) -> Tuple[Vec3, Mat3]:
    """Return flange->TCP translation and rotation for the selected tool."""
    if str(tool_id) == "suction_gripper":
        rotation = SUCTION_TOOL_ROTATION
        # The official pump URDF places the head frame 10 mm along flange Z.
        # The directly measured flange-to-contact length remains authoritative,
        # so that 10 mm is part of (not added to) the installed contact length.
        remaining = max(0.0, float(suction_contact_distance_m) - SUCTION_MOUNT_TRANSLATION[2])
        vector = _vec_add(
            SUCTION_MOUNT_TRANSLATION,
            _mat_vec(rotation, (0.0, remaining, 0.0)),
        )
    else:
        rotation = ADAPTIVE_TOOL_ROTATION
        vector = ADAPTIVE_TOOL_TCP_VECTOR
    correction = correction_local_m or (0.0, 0.0, 0.0)
    if len(correction) < 3:
        raise ValueError("tool correction must contain x, y, z")
    vector = _vec_add(vector, _mat_vec(rotation, correction))
    return vector, rotation


def forward_flange_kinematics(angles_deg: Sequence[float]) -> Tuple[Vec3, Mat3]:
    """Robot flange pose in the base frame for six physical joint angles."""
    if len(angles_deg) != 6:
        raise ValueError("forward_flange_kinematics requires 6 joint angles")
    rotation: Mat3 = IDENTITY
    px = py = pz = 0.0
    radians = math.radians
    cos, sin = math.cos, math.sin
    for index, (translation, static_rot, offset_rad) in enumerate(_CHAIN_FAST):
        tx, ty, tz = translation
        if tx or ty or tz:
            r0, r1, r2 = rotation
            px += r0[0] * tx + r0[1] * ty + r0[2] * tz
            py += r1[0] * tx + r1[1] * ty + r1[2] * tz
            pz += r2[0] * tx + r2[1] * ty + r2[2] * tz
        theta = radians(float(angles_deg[index])) + offset_rad
        c, s = cos(theta), sin(theta)
        # static_rot . Rz(theta): only the first two columns mix.
        (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = static_rot
        local = (
            (m00 * c + m01 * s, -m00 * s + m01 * c, m02),
            (m10 * c + m11 * s, -m10 * s + m11 * c, m12),
            (m20 * c + m21 * s, -m20 * s + m21 * c, m22),
        )
        rotation = _mat_mul(rotation, local)

    return (px, py, pz), rotation


def firmware_flange_kinematics(angles_deg: Sequence[float]) -> Tuple[Vec3, Mat3]:
    """Flange pose expressed in the myCobot firmware coordinate frame."""
    position, rotation = forward_flange_kinematics(angles_deg)
    return _vec_add(position, FIRMWARE_BASE_TRANSLATION_M), rotation


def tcp_from_flange(
    position: Sequence[float], rotation: Mat3,
    tool_id: str = "adaptive_gripper",
    correction_local_m: Optional[Sequence[float]] = None,
    suction_contact_distance_m: float = SUCTION_CONTACT_DISTANCE_M,
) -> Tuple[Vec3, Mat3]:
    """Apply the selected, calibrated flange->TCP transform."""
    vector, tool_rotation = tool_transform(
        tool_id, correction_local_m, suction_contact_distance_m
    )
    tcp_rotation = _mat_mul(rotation, tool_rotation)
    tcp_position = _vec_add(position, _mat_vec(rotation, vector))
    return tcp_position, tcp_rotation


def flange_from_tcp(
    position: Sequence[float], rotation: Mat3,
    tool_id: str = "adaptive_gripper",
    correction_local_m: Optional[Sequence[float]] = None,
    suction_contact_distance_m: float = SUCTION_CONTACT_DISTANCE_M,
) -> Tuple[Vec3, Mat3]:
    """Invert :func:`tcp_from_flange` for an exact contact target."""
    vector, tool_rotation = tool_transform(
        tool_id, correction_local_m, suction_contact_distance_m
    )
    flange_rotation = _mat_mul(rotation, _mat_t(tool_rotation))
    flange_position = _vec_sub(position, _mat_vec(flange_rotation, vector))
    return flange_position, flange_rotation


def top_down_tcp_rotation(jaw_yaw_deg: float) -> Mat3:
    """Canonical TCP axes: jaw axis horizontal and approach axis straight down."""
    return grasp_rotation(float(jaw_yaw_deg), 0.0, 0.0)


def top_down_flange_pose(
    position: Sequence[float], jaw_yaw_deg: float,
    tool_id: str = "adaptive_gripper",
    correction_local_m: Optional[Sequence[float]] = None,
    suction_contact_distance_m: float = SUCTION_CONTACT_DISTANCE_M,
) -> Tuple[Vec3, Mat3]:
    """Exact flange pose that places the selected TCP at ``position``."""
    return flange_from_tcp(
        position, top_down_tcp_rotation(jaw_yaw_deg), tool_id,
        correction_local_m, suction_contact_distance_m,
    )


def tool_axis_diagnostics(
    flange_rotation: Mat3, tool_id: str = "adaptive_gripper"
) -> Dict[str, object]:
    """Return matrix-based contact/approach diagnostics for a flange orientation."""
    _, tool_rotation = tool_transform(tool_id)
    tcp_rotation = _mat_mul(flange_rotation, tool_rotation)
    jaw_axis = (tcp_rotation[0][0], tcp_rotation[1][0], tcp_rotation[2][0])
    approach_axis = (tcp_rotation[0][1], tcp_rotation[1][1], tcp_rotation[2][1])
    down_alignment = max(-1.0, min(1.0, -approach_axis[2]))
    tilt_deg = math.degrees(math.acos(down_alignment))
    jaw_yaw_deg = math.degrees(math.atan2(jaw_axis[1], jaw_axis[0]))
    return {
        "jawAxis": jaw_axis,
        "approachAxis": approach_axis,
        "jawYawDeg": ((jaw_yaw_deg + 180.0) % 360.0) - 180.0,
        "approachTiltDeg": tilt_deg,
        "topDown": tilt_deg <= MAX_TOP_DOWN_TILT_DEG,
    }


def forward_kinematics(
    angles_deg: Sequence[float], tool_id: str = "adaptive_gripper",
    correction_local_m: Optional[Sequence[float]] = None,
    suction_contact_distance_m: float = SUCTION_CONTACT_DISTANCE_M,
) -> Tuple[Vec3, Mat3]:
    """Selected tool TCP pose in the robot base frame."""
    return tcp_from_flange(
        *forward_flange_kinematics(angles_deg), tool_id,
        correction_local_m, suction_contact_distance_m,
    )


def rotation_from_rpy_deg(rpy_deg: Sequence[float]) -> Mat3:
    """Firmware XYZ/RPY orientation as Rz(yaw) * Ry(pitch) * Rx(roll)."""
    if len(rpy_deg) < 3:
        raise ValueError("rotation_from_rpy_deg requires rx, ry, rz")
    rx, ry, rz = (math.radians(float(value)) for value in rpy_deg[:3])
    return _mat_mul(_rotz(rz), _mat_mul(_roty(ry), _rotx(rx)))


def rpy_deg_from_rotation(rotation: Mat3) -> Vec3:
    """Inverse of rotation_from_rpy_deg, with the usual gimbal-lock branch."""
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0][1], rotation[1][1])
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def pose_residual(
    actual_pos: Sequence[float], actual_rot: Mat3,
    target_pos: Sequence[float], target_rot: Mat3,
) -> Tuple[float, float]:
    """Return position residual in meters and orientation residual in radians."""
    return _vec_norm(_vec_sub(target_pos, actual_pos)), _vec_norm(_rotation_error(target_rot, actual_rot))


def tool_axis(rotation: Mat3) -> Vec3:
    """World direction the gripper fingers point (gripper local +Y)."""
    return _mat_vec(rotation, (0.0, 1.0, 0.0))


def _rotation_error(target: Mat3, current: Mat3) -> Vec3:
    """Rotation vector (axis * angle) taking current to target."""
    err = _mat_mul(target, _mat_t(current))
    trace = err[0][0] + err[1][1] + err[2][2]
    cos_a = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    angle = math.acos(cos_a)
    if angle < 1e-9:
        return (0.0, 0.0, 0.0)
    if angle > math.pi - 1e-4:
        # Near pi: fall back to the dominant diagonal axis.
        axis = max(range(3), key=lambda i: err[i][i])
        v = [0.0, 0.0, 0.0]
        v[axis] = angle
        return (v[0], v[1], v[2])
    scale = angle / (2.0 * math.sin(angle))
    return (
        (err[2][1] - err[1][2]) * scale,
        (err[0][2] - err[2][0]) * scale,
        (err[1][0] - err[0][1]) * scale,
    )


def _solve_6x6(matrix: List[List[float]], rhs: List[float]) -> Optional[List[float]]:
    n = 6
    a = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-12:
            return None
        a[col], a[pivot] = a[pivot], a[col]
        inv = 1.0 / a[col][col]
        for r in range(n):
            if r == col:
                continue
            factor = a[r][col] * inv
            if factor == 0.0:
                continue
            for c in range(col, n + 1):
                a[r][c] -= factor * a[col][c]
    return [a[i][n] / a[i][i] for i in range(n)]


def _clamp_to_limits(angles: List[float]) -> List[float]:
    out = []
    for index, value in enumerate(angles):
        low, high = JOINT_LIMITS_DEG[index + 1]
        out.append(max(low, min(high, value)))
    return out


def _pose_error(
    angles_deg: Sequence[float], target_pos: Vec3, target_rot: Mat3,
    tool_id: str = "adaptive_gripper",
    correction_local_m: Optional[Sequence[float]] = None,
    suction_contact_distance_m: float = SUCTION_CONTACT_DISTANCE_M,
) -> Tuple[List[float], float, float]:
    pos, rot = forward_kinematics(
        angles_deg, tool_id, correction_local_m, suction_contact_distance_m
    )
    e_pos = _vec_sub(target_pos, pos)
    e_rot = _rotation_error(target_rot, rot)
    error = [e_pos[0], e_pos[1], e_pos[2], e_rot[0], e_rot[1], e_rot[2]]
    return error, _vec_norm(e_pos), _vec_norm(e_rot)


def _solve_pose_seed(
    target_pos: Sequence[float],
    target_rot: Mat3,
    seed_deg: Sequence[float],
    max_iterations: int = IK_MAX_ITERATIONS,
    tool_id: str = "adaptive_gripper",
    correction_local_m: Optional[Sequence[float]] = None,
    suction_contact_distance_m: float = SUCTION_CONTACT_DISTANCE_M,
    diagnostics: Optional[dict] = None,
) -> Optional[List[float]]:
    """Damped least-squares IK for a full 6-DOF pose. Returns degrees or None."""
    angles = _clamp_to_limits([float(v) for v in seed_deg])
    target = (float(target_pos[0]), float(target_pos[1]), float(target_pos[2]))
    eps_deg = 0.25
    best_metric = float("inf")
    best_angles = list(angles)
    stalled = 0

    iterations = 0
    for _ in range(max_iterations):
        iterations += 1
        error, pos_err, rot_err = _pose_error(
            angles, target, target_rot, tool_id,
            correction_local_m, suction_contact_distance_m,
        )
        if pos_err <= IK_POSITION_TOLERANCE_M and rot_err <= IK_ORIENTATION_TOLERANCE_RAD:
            if diagnostics is not None:
                diagnostics.update({
                    "ok": True, "bestAngles": list(angles),
                    "positionErrorM": pos_err, "orientationErrorRad": rot_err,
                    "iterations": iterations,
                })
            return [round(v, 5) for v in angles]
        metric = pos_err + 0.05 * rot_err
        if metric < best_metric - 1e-6:
            best_metric = metric
            best_angles = list(angles)
            stalled = 0
        else:
            stalled += 1
            if stalled >= 24:
                break

        # Numeric Jacobian of the pose (not the error): d f / d theta_j.
        # error = target - f, so d(error)/d(theta) = -J and the DLS update
        # delta = J^T (J J^T + lambda^2 I)^-1 error needs J, not -J.
        jacobian: List[List[float]] = [[0.0] * 6 for _ in range(6)]
        for j in range(6):
            bumped = list(angles)
            bumped[j] += eps_deg
            bumped_error, _, _ = _pose_error(
                bumped, target, target_rot, tool_id,
                correction_local_m, suction_contact_distance_m,
            )
            inv_step = 1.0 / math.radians(eps_deg)
            for i in range(6):
                jacobian[i][j] = (error[i] - bumped_error[i]) * inv_step

        # Delta = J^T (J J^T + lambda^2 I)^-1 error
        jjt = [
            [
                sum(jacobian[i][k] * jacobian[r][k] for k in range(6))
                + (IK_DAMPING * IK_DAMPING if i == r else 0.0)
                for r in range(6)
            ]
            for i in range(6)
        ]
        intermediate = _solve_6x6(jjt, error)
        if intermediate is None:
            return None
        delta = [
            sum(jacobian[i][j] * intermediate[i] for i in range(6)) for j in range(6)
        ]
        largest = max(abs(d) for d in delta)
        if largest > IK_MAX_STEP_RAD:
            scale = IK_MAX_STEP_RAD / largest
            delta = [d * scale for d in delta]
        angles = _clamp_to_limits(
            [angles[j] + math.degrees(delta[j]) for j in range(6)]
        )

    error, pos_err, rot_err = _pose_error(
        best_angles, target, target_rot, tool_id,
        correction_local_m, suction_contact_distance_m,
    )
    if pos_err <= IK_POSITION_TOLERANCE_M and rot_err <= IK_ORIENTATION_TOLERANCE_RAD:
        if diagnostics is not None:
            diagnostics.update({
                "ok": True, "bestAngles": list(best_angles),
                "positionErrorM": pos_err, "orientationErrorRad": rot_err,
                "iterations": iterations,
            })
        return [round(v, 5) for v in best_angles]
    if diagnostics is not None:
        diagnostics.update({
            "ok": False, "bestAngles": list(best_angles),
            "positionErrorM": pos_err, "orientationErrorRad": rot_err,
            "iterations": iterations,
        })
    return None


def solve_pose(
    target_pos: Sequence[float],
    target_rot: Mat3,
    seed_deg: Sequence[float],
    max_iterations: int = IK_MAX_ITERATIONS,
    exhaustive: bool = True,
    tool_id: str = "adaptive_gripper",
    correction_local_m: Optional[Sequence[float]] = None,
    suction_contact_distance_m: float = SUCTION_CONTACT_DISTANCE_M,
    diagnostics: Optional[dict] = None,
) -> Optional[List[float]]:
    """Multi-start damped least-squares IK using deterministic, bounded seeds."""
    x, y, z = (float(value) for value in target_pos[:3])
    primary = _clamp_to_limits([float(v) for v in seed_deg])
    geometric = _geometric_seed(x, y, z)
    seeds = [
        primary,
        geometric,
        [geometric[0], -geometric[1], -geometric[2], -geometric[3], 0.0, geometric[5]],
    ]
    # Wrist/elbow alternatives cover the common local minima while remaining
    # deterministic and fast enough for preview-time use.
    for j3 in (-90.0, 0.0, 90.0):
        for j5 in (-90.0, 0.0, 90.0):
            seeds.append([geometric[0], geometric[1], j3, geometric[3], j5, geometric[5]])
    seen = set()
    best_seed_result: Optional[dict] = None
    attempts = 0

    def record(result: dict, phase: str) -> None:
        nonlocal best_seed_result, attempts
        attempts += 1
        raw_position_error = result.get("positionErrorM")
        raw_orientation_error = result.get("orientationErrorRad")
        position_error = (
            float(raw_position_error) if raw_position_error is not None else float("inf")
        )
        orientation_error = (
            float(raw_orientation_error) if raw_orientation_error is not None else float("inf")
        )
        metric = position_error + 0.05 * orientation_error
        result["metric"] = metric
        result["phase"] = phase
        if best_seed_result is None or metric < float(best_seed_result.get("metric", float("inf"))):
            best_seed_result = dict(result)

    def finish(solution: Optional[List[float]], phase: str) -> Optional[List[float]]:
        if diagnostics is not None:
            diagnostics.update({
                "ok": solution is not None,
                "angles": list(solution) if solution is not None else None,
                "seedsAttempted": attempts,
                "failurePhase": None if solution is not None else phase,
                "bestAngles": (best_seed_result or {}).get("bestAngles"),
                "bestPositionErrorM": (best_seed_result or {}).get("positionErrorM"),
                "bestOrientationErrorRad": (best_seed_result or {}).get("orientationErrorRad"),
            })
        return solution

    for seed in seeds:
        key = tuple(round(v, 3) for v in seed)
        if key in seen:
            continue
        seen.add(key)
        seed_result: dict = {}
        solution = _solve_pose_seed(
            target_pos, target_rot, seed, max_iterations, tool_id,
            correction_local_m, suction_contact_distance_m,
            diagnostics=seed_result,
        )
        record(seed_result, "fast")
        if solution is not None:
            return finish(solution, "fast")
    if not exhaustive:
        return finish(None, "fast")
    # Exhaustive fallback for difficult wrist branches. This runs only after
    # the fast seeds fail; the grid is deliberately coarse and deterministic.
    base = geometric[0]
    # J1 is geometrically determined by the target bearing. The previous
    # +/-180-degree variants clamp against the J1 limits and multiply every
    # unreachable solve by three without exposing a useful physical branch.
    for shoulder in (-100.0, -50.0, 0.0, 50.0, 100.0):
        for elbow in (-120.0, -60.0, 0.0, 60.0, 120.0):
            for wrist in (-120.0, 0.0, 120.0):
                    seed = _clamp_to_limits([base, shoulder, elbow, 0.0, wrist, -45.0])
                    seed_result = {}
                    solution = _solve_pose_seed(
                        target_pos, target_rot, seed, max_iterations, tool_id,
                        correction_local_m, suction_contact_distance_m,
                        diagnostics=seed_result,
                    )
                    record(seed_result, "exhaustive")
                    if solution is not None:
                        return finish(solution, "exhaustive")
    return finish(None, "exhaustive")


def solve_pose_diagnostics(
    target_pos: Sequence[float], target_rot: Mat3, seed_deg: Sequence[float],
    max_iterations: int = IK_MAX_ITERATIONS, exhaustive: bool = True,
    tool_id: str = "adaptive_gripper",
    correction_local_m: Optional[Sequence[float]] = None,
    suction_contact_distance_m: float = SUCTION_CONTACT_DISTANCE_M,
) -> dict:
    result: dict = {}
    solve_pose(
        target_pos, target_rot, seed_deg, max_iterations=max_iterations,
        exhaustive=exhaustive, tool_id=tool_id,
        correction_local_m=correction_local_m,
        suction_contact_distance_m=suction_contact_distance_m,
        diagnostics=result,
    )
    return result


def grasp_rotation(yaw_deg: float, tilt_deg: float, radial_yaw_rad: float) -> Mat3:
    """
    Tool orientation for a top-down grasp.

    yaw_deg: finger-opening direction in the horizontal plane.
    tilt_deg: lean of the tool away from vertical. Positive tilt points the
        fingertips outward (away from the base), which moves the flange
        inward and extends usable reach at the workspace edge.
    radial_yaw_rad: bearing of the grasp point seen from the base (atan2(y, x)).
    """
    down: Vec3 = (0.0, 0.0, -1.0)
    if abs(tilt_deg) > 1e-6:
        # Rotate about the horizontal tangent so `down` gains a +radial part.
        tangent = (math.sin(radial_yaw_rad), -math.cos(radial_yaw_rad), 0.0)
        tilt = math.radians(tilt_deg)
        c, s = math.cos(tilt), math.sin(tilt)
        k = tangent
        # Rodrigues rotation of `down` about `tangent`.
        kv = _vec_cross(k, down)
        kkv = _vec_cross(k, kv)
        down = _vec_add(_vec_add(_vec_scale(down, c), _vec_scale(kv, s)), _vec_scale(kkv, 1.0 - c))
        down = _vec_unit(down)

    yaw = math.radians(yaw_deg)
    finger = (math.cos(yaw), math.sin(yaw), 0.0)
    # Make the finger axis orthogonal to the (tilted) approach axis.
    dot = finger[0] * down[0] + finger[1] * down[1] + finger[2] * down[2]
    finger = _vec_unit(_vec_sub(finger, _vec_scale(down, dot)))
    z_axis = _vec_cross(finger, down)
    return (
        (finger[0], down[0], z_axis[0]),
        (finger[1], down[1], z_axis[1]),
        (finger[2], down[2], z_axis[2]),
    )


def _geometric_seed(x: float, y: float, z: float) -> List[float]:
    """Rough elbow configuration used to start the numeric solver."""
    base = math.degrees(math.atan2(y, x))
    radial = max(0.001, math.hypot(x, y) - 0.035)
    shoulder_z = z + 0.12 - 0.13156  # aim the wrist region above the grasp
    upper, forearm = 0.1104, 0.096
    target_r = max(0.001, radial - 0.02)
    dist = max(0.04, min(math.hypot(target_r, shoulder_z), upper + forearm - 0.005))
    cos_elbow = max(-1.0, min(1.0, (dist * dist - upper * upper - forearm * forearm) / (2 * upper * forearm)))
    elbow = math.acos(cos_elbow)
    shoulder = math.atan2(shoulder_z, target_r) - math.atan2(
        forearm * math.sin(elbow), upper + forearm * math.cos(elbow)
    )
    j2 = max(-120.0, min(100.0, math.degrees(shoulder)))
    j3 = max(-120.0, min(135.0, 100.0 - math.degrees(elbow)))
    return [base, j2, j3, -(j2 + j3) - 90.0, 0.0, -45.0]


def solve_top_down(
    position: Sequence[float],
    yaw_deg: float = 0.0,
    seed_deg: Optional[Sequence[float]] = None,
) -> Optional[Dict[str, object]]:
    """
    Solve a top-down grasp pose at `position`, fingers opening along `yaw_deg`.

    Tries a perfectly vertical tool first, then progressively tilted approaches,
    and equivalent yaws (boxes are symmetric under 90/180 degree rotations).
    Returns {"angles", "tiltDeg", "yawDeg", "positionErrorM"} or None.
    """
    x, y, z = float(position[0]), float(position[1]), float(position[2])
    # Cheap envelope check before burning solver time: the 280's reach is
    # ~280 mm from the J2 axis; the ~113 mm tool can extend that when tilted.
    radial = math.hypot(x, y)
    if radial > 0.33 or z < -0.01 or z > 0.38:
        return None
    if math.hypot(radial, z - 0.13156) > 0.30 + 0.113:
        return None
    radial_yaw = math.atan2(y, x) if (abs(x) > 1e-9 or abs(y) > 1e-9) else 0.0
    seeds: List[List[float]] = []
    if seed_deg is not None:
        seeds.append([float(v) for v in seed_deg])
    seeds.append(_geometric_seed(x, y, z))

    for tilt in APPROACH_TILTS_DEG:
        for yaw_option in (yaw_deg, yaw_deg + 90.0, yaw_deg - 90.0, yaw_deg + 180.0):
            rotation = grasp_rotation(yaw_option, tilt, radial_yaw)
            for seed in seeds:
                solution = solve_pose((x, y, z), rotation, seed)
                if solution is None:
                    continue
                pos, _ = forward_kinematics(solution)
                return {
                    "angles": solution,
                    "tiltDeg": tilt,
                    "yawDeg": round(((yaw_option + 180.0) % 360.0) - 180.0, 2),
                    "positionErrorM": round(_vec_norm(_vec_sub((x, y, z), pos)), 5),
                }
    return None


def solve_vertical_segment_candidates(
    xy: Sequence[float],
    z_top: float,
    z_bottom: float,
    yaw_deg: float,
    seed_deg: Optional[Sequence[float]] = None,
    yaw_offsets_deg: Sequence[float] = (0.0, 90.0, -90.0, 180.0),
) -> List[Dict[str, object]]:
    """
    Find every vertical-segment candidate that solves both ends with one
    constant tool orientation. The caller can score candidates for physical
    friendliness instead of taking the first numeric IK solution.
    """
    x, y = float(xy[0]), float(xy[1])
    radial = math.hypot(x, y)
    if radial > 0.33:
        return []
    radial_yaw = math.atan2(y, x) if (abs(x) > 1e-9 or abs(y) > 1e-9) else 0.0
    seeds: List[List[float]] = []
    if seed_deg is not None:
        seeds.append([float(v) for v in seed_deg])
    seeds.append(_geometric_seed(x, y, float(z_bottom)))

    candidates: List[Dict[str, object]] = []
    for tilt in APPROACH_TILTS_DEG:
        for yaw_offset in yaw_offsets_deg:
            yaw_option = yaw_deg + float(yaw_offset)
            rotation = grasp_rotation(yaw_option, tilt, radial_yaw)
            for seed_index, seed in enumerate(seeds):
                bottom = solve_pose((x, y, float(z_bottom)), rotation, seed)
                if bottom is None:
                    continue
                top = solve_pose((x, y, float(z_top)), rotation, bottom)
                if top is None:
                    continue
                candidates.append({
                    "rotation": rotation,
                    "tiltDeg": tilt,
                    "yawOffsetDeg": round(float(yaw_offset), 2),
                    "yawDeg": round(((yaw_option + 180.0) % 360.0) - 180.0, 2),
                    "top": top,
                    "bottom": bottom,
                    "seedIndex": seed_index,
                    "candidateIndex": len(candidates),
                })
    return candidates


def solve_vertical_segment(
    xy: Sequence[float],
    z_top: float,
    z_bottom: float,
    yaw_deg: float,
    seed_deg: Optional[Sequence[float]] = None,
) -> Optional[Dict[str, object]]:
    """
    Find one grasp rotation (tilt + yaw) that solves BOTH ends of a vertical
    segment, so the descend/lift between them can keep a constant tool
    orientation. The top of the segment is usually the constraining pose.
    Returns {"rotation", "tiltDeg", "yawDeg", "top", "bottom"} or None.
    """
    candidates = solve_vertical_segment_candidates(xy, z_top, z_bottom, yaw_deg, seed_deg)
    return candidates[0] if candidates else None


def cartesian_descent(
    start: Sequence[float],
    end: Sequence[float],
    rotation: Mat3,
    seed_deg: Sequence[float],
    step_m: float = 0.03,
    max_waypoints: int = 3,
) -> Optional[List[List[float]]]:
    """
    IK along a straight Cartesian segment (used for vertical descend/lift).
    Each waypoint seeds the next, keeping the joint path continuous.
    Returns the list of waypoint joint angles, or None if any waypoint fails.
    """
    delta = _vec_sub(end, start)
    length = _vec_norm(delta)
    count = max(1, int(math.ceil(length / max(0.004, step_m))))
    count = min(max(1, int(max_waypoints)), count)
    waypoints: List[List[float]] = []
    seed = [float(v) for v in seed_deg]
    for i in range(1, count + 1):
        t = i / count
        point = _vec_add(start, _vec_scale(delta, t))
        solution = solve_pose(point, rotation, seed, max_iterations=80)
        if solution is None:
            return None
        waypoints.append(solution)
        seed = solution
    return waypoints


def joint_space_path(
    start_deg: Sequence[float],
    end_deg: Sequence[float],
    max_step_deg: float = 18.0,
) -> List[List[float]]:
    """Evenly spaced joint-space via points for smooth blended transit moves."""
    deltas = [
        ((float(e) - float(s) + 180.0) % 360.0) - 180.0
        for s, e in zip(start_deg, end_deg)
    ]
    largest = max(abs(d) for d in deltas) if deltas else 0.0
    count = max(1, int(math.ceil(largest / max(4.0, max_step_deg))))
    path = []
    for i in range(1, count + 1):
        t = i / count
        path.append([round(float(s) + d * t, 3) for s, d in zip(start_deg, deltas)])
    return path
