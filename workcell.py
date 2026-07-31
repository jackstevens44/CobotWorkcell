#!/usr/bin/env python3
"""
Workcell model for the myCobot 280 dashboard: parts, parametric bins,
pick/place programs, camera calibration, and the coordinate planner.

Everything lives in one JSON file (data/workcell.json) so the cell you build
in the browser survives server restarts.

Frames: robot base, meters, +X forward, +Y left, +Z up. Table top is z=0.
Part positions are the part's center; bin positions are the center of the
bin's floor footprint (base resting on the table).

Camera localization is fiducial-only: workspace tags establish the robot table
frame and registered object tags provide deterministic live part poses.
"""

from __future__ import annotations

import json
import hashlib
import math
import os
import threading
import time
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from mycobot_kinematics import (
    JOINT_LIMITS_DEG,
    flange_from_tcp,
    pose_residual,
    rotation_from_rpy_deg,
    rpy_deg_from_rotation,
    tcp_from_flange,
    tool_axis_diagnostics,
    top_down_tcp_rotation,
    top_down_flange_pose,
)

SCENE_BOUND_M = 0.42
TABLE_Z = 0.0

# Usable vertical contact length below the modeled jaw-center TCP.  In a
# top-down side pinch the finger contact segment spans
# [jaw_center_z - length, jaw_center_z].
ADAPTIVE_GRIPPER_FINGER_CONTACT_LENGTH_M = 0.021
DEFAULT_TABLE_CLEARANCE_M = 0.004
SIDE_PINCH_CAPTURE_ABOVE_POCKET_M = 0.014
SIDE_PINCH_XY_TOLERANCE_M = 0.018
SIDE_PINCH_YAW_TOLERANCE_DEG = 35.0
# A top-down side pinch changes the jaw axis, not the XY center of the TCP.
# The previous 10 mm face-directed inset made both simulated and physical
# picks visibly miss the object center.
SURFACE_GRIP_DEPTH_M = 0.0
SURFACE_CENTER_BAND_RATIO = 0.70
GRIPPER_MAX_SIDE_PINCH_WIDTH_M = 0.075
GRIPPER_MIN_SIDE_PINCH_WIDTH_M = 0.008

PREGRASP_RISE_M = 0.04
TRANSIT_EXTRA_CLEARANCE_M = 0.02
BIN_APPROACH_CLEARANCE_M = 0.02
BIN_RELEASE_CLEARANCE_M = 0.015
BIN_DROP_WALL_CLEARANCE_M = 0.006
RELEASE_DROP_GAP_M = 0.015
MIN_TRANSIT_Z = 0.12

# Angular firmware moves still need a continuous joint branch.  A single
# cross-table carry can be Cartesian-reachable at both ends while asking J1 to
# jump more than the preview's conservative continuity limit.  Split those
# transfers into real commanded waypoints; do not weaken IK validation or
# silently move the requested pick/drop positions.
MAX_TRANSFER_XY_LEG_M = 0.160
MAX_TRANSFER_BEARING_STEP_DEG = 55.0
# Keep the carried TCP/object route outside the robot pedestal and attached
# base hardware.  This is a path-planning clearance, not an IK reach limit.
BASE_TRANSFER_CLEARANCE_RADIUS_M = 0.130
SPATIAL_PLACEMENT_MARGIN_M = 0.010
SPATIAL_GRID_STEP_M = 0.015
# Enter a low pickup from a top-down staging pose.  This gives the firmware a
# nearby joint branch before the short final approach instead of asking it to
# unfold from HOME directly into the low target pose.
APPROACH_STAGING_Z_M = 0.160
# A 160 mm-high top-down TCP staging pose loses horizontal reach before the
# lower pickup pose does. Beyond this radius, routing through the fixed high
# staging point creates a false "unreachable" plan for an otherwise reachable
# outer-workspace suction pick. The normal approach already provides the safe
# vertical clearance and is sent in angular coordinate mode.
APPROACH_STAGING_MAX_RADIUS_M = 0.230

SPEED_TRANSIT = 20
SPEED_LIFT = 16
# Descend/lower run as a single firmware linear move now, so the old crawl speed
# (12) that made the servos cog is gone; a steady moderate speed is smoother.
# Lower this if the approach onto the object feels too quick.
SPEED_DESCEND = 30

# J6 rests at the gripper's neutral mount angle (-45) so "home" leaves the
# gripper square; J1-J5 stay at 0. These are true physical angles (the same
# convention get_angles/send_angles use).
HOME_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0, -45.0]
GRIPPER_MOUNT_NEUTRAL_J6_DEG = -45.0
SURFACE_GRASP_YAW_OFFSETS_DEG = (0.0, 180.0)

PHYSICAL_CONFIRM_TOKEN = "RUN_PHYSICAL_PICK"
END_EFFECTORS = {
    "adaptive_gripper": "Adaptive Gripper",
    "suction_gripper": "Air Suction Gripper",
}

TOOL_TCP_OFFSETS_M = {
    "adaptive_gripper": {"x": 0.0, "y": 0.078, "z": 0.0},
    "suction_gripper": {"x": 0.0, "y": 0.072, "z": 0.0},
}

DEFAULT_TOOL_PROFILES = {
    "adaptive_gripper": {
        "tcpCorrectionLocalM": {"x": 0.0, "y": 0.0, "z": 0.0},
        "geometry": {
            "contactType": "jaw_pocket",
            "fingerContactLengthM": ADAPTIVE_GRIPPER_FINGER_CONTACT_LENGTH_M,
        },
    },
    "suction_gripper": {
        "tcpCorrectionLocalM": {"x": 0.0, "y": 0.0, "z": 0.0},
        "geometry": {
            "contactType": "single_suction_cup",
            "flangeToCupStartM": 0.050,
            "cupFreeExtensionM": 0.022,
            "flangeToContactM": 0.072,
            "cupDiameterM": 0.022,
        },
        "hardware": {
            "pumpBoxNominalM": {"x": 0.072, "y": 0.052, "z": 0.037},
            "wristHeadNominalM": {"x": 0.063, "y": 0.0245, "z": 0.0267},
            "totalAccessoryMassKg": 0.180,
            "ratedPayloadKg": 0.150,
            "physicalCenterOfMassM": None,
            "centerOfMassStatus": "unknown_not_published",
            "basePumpExcludedFromMovingPayload": True,
            "wiringProfile": "pump_v2",
            "requiresSeparatePumpPower": True,
            "baseIoPinMap": {
                "GND": "GND", "5V": "5V", "G2": 2, "G5": 5,
            },
        },
    },
}

DEFAULT_PICKUP_PROFILES = {
    "adaptive_gripper": {
        "offsetLocalM": {"x": 0.0, "y": 0.0, "z": 0.0},
        "jawYawMode": "automatic_narrow_side",
        "jawYawOverrideDeg": None,
        "maximumTiltDeg": 10.0,
    },
    "suction_gripper": {
        "offsetLocalM": {"x": 0.0, "y": 0.0, "z": 0.0},
        "contactPreloadM": 0.002,
        "yawMode": "minimum_joint_travel",
    },
}

COORD_MODE_ANGULAR = 0
COORD_MODE_LINEAR = 1

MYCOBOT_280_COORD_LIMITS = [
    {"axis": "x", "min": -281.45, "max": 281.45, "unit": "mm"},
    {"axis": "y", "min": -281.45, "max": 281.45, "unit": "mm"},
    {"axis": "z", "min": -70.0, "max": 412.67, "unit": "mm"},
    {"axis": "rx", "min": -180.0, "max": 180.0, "unit": "deg"},
    {"axis": "ry", "min": -180.0, "max": 180.0, "unit": "deg"},
    {"axis": "rz", "min": -180.0, "max": 180.0, "unit": "deg"},
]


def validate_coordinate_bounds(
    coords: Any,
    state_id: str = "unknown",
    allow_missing_rpy: bool = True,
) -> List[Dict[str, Any]]:
    errors: List[Dict[str, Any]] = []
    if not isinstance(coords, list) or len(coords) != 6:
        return [{
            "stateId": state_id,
            "axis": "coords",
            "error": "coordsMm must be [x,y,z,rx,ry,rz].",
        }]
    for index, limit in enumerate(MYCOBOT_280_COORD_LIMITS):
        value = coords[index]
        if value is None and allow_missing_rpy and index >= 3:
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            errors.append({
                "stateId": state_id,
                "axis": limit["axis"],
                "value": value,
                "min": limit["min"],
                "max": limit["max"],
                "unit": limit["unit"],
                "error": "non_numeric_coordinate",
            })
            continue
        if number < float(limit["min"]) or number > float(limit["max"]):
            errors.append({
                "stateId": state_id,
                "axis": limit["axis"],
                "value": round(number, 3),
                "min": limit["min"],
                "max": limit["max"],
                "unit": limit["unit"],
                "error": "coordinate_out_of_bounds",
                "message": (
                    f"State {state_id} has invalid coord value on {limit['axis']}: "
                    f"{round(number, 3)} {limit['unit']} outside "
                    f"{limit['min']} ~ {limit['max']} {limit['unit']}."
                ),
            })
    return errors


def clamp(value: Any, low: float, high: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = low
    return max(low, min(high, number))


def _wrap_deg(value: float) -> float:
    return ((float(value) + 180.0) % 360.0) - 180.0


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


class Workcell:
    def __init__(self, data_dir: Path) -> None:
        # Several high-level operations intentionally compose smaller scene
        # helpers while holding the scene lock.  Use an RLock so those helpers
        # can also protect direct callers without deadlocking the server.
        self.lock = threading.RLock()
        self.data_dir = data_dir
        self.path = data_dir / "workcell.json"
        self.parts: Dict[str, Dict[str, Any]] = {}
        self.registered_parts: Dict[str, Dict[str, Any]] = {}
        self.tag_track_revision = 0
        self.tag_last_seen: Dict[str, float] = {}
        self.bins: Dict[str, Dict[str, Any]] = {}
        self.taught_points: Dict[str, Dict[str, Any]] = {}
        self.programs: Dict[str, Dict[str, Any]] = {}
        self.calibration: Dict[str, Any] = self._default_calibration()
        self.camera: Dict[str, Any] = self._default_camera()
        self.coordinate_planner: Dict[str, Any] = self._default_coordinate_planner()
        self.end_effector = "adaptive_gripper"
        self.version = 0
        self.updated_at: Optional[float] = None
        self._counter = 1
        self._load()

    # ------------------------------------------------------------- storage

    @staticmethod
    def _default_calibration() -> Dict[str, Any]:
        return {
            "status": "not_configured",
            "cameraToRobot": None,
            "note": (
                "AprilTag workspace markers define the camera-to-robot table "
                "mapping. Legacy cameraToRobot data is retained only for "
                "backward-compatible loading and visualization."
            ),
            "updatedAt": None,
            "intrinsics": None,
            "fiducials": {
                "dictionary": "DICT_APRILTAG_36h11",
                "markerSizeM": 0.05,
                "minimumMarkers": 3,
                "validationProfile": "practical",
                "maxReprojectionRmsPx": 10.0,
                "maxReprojectionPx": 18.0,
                "referenceMarkers": [],
                "objectTags": [],
                "baselineHomography": None,
            },
            "verification": None,
        }

    @staticmethod
    def _default_camera() -> Dict[str, Any]:
        return {
            "deviceId": 0,
            "deviceUniqueId": None,
            "deviceLabel": None,
            "devicePolicy": "external_only",
            "enabled": False,
            "width": 1280,
            "height": 720,
            "jpegQuality": 82,
            "staleAfterS": 3.0,
            "localization": {
                "enabled": False,
                "intervalS": 0.08,
            },
            "workspaceBounds": {
                "xMin": -SCENE_BOUND_M,
                "xMax": SCENE_BOUND_M,
                "yMin": -SCENE_BOUND_M,
                "yMax": SCENE_BOUND_M,
                "zMin": TABLE_Z,
                "zMax": 0.35,
            },
            "updatedAt": None,
        }

    @staticmethod
    def _default_coordinate_planner() -> Dict[str, Any]:
        return {
            "toolRpyDeg": None,
            "toolRpySource": "canonical_top_down",
            "pickHeightBiasM": 0.0,
            "minimumTableClearanceM": DEFAULT_TABLE_CLEARANCE_M,
            "toolOffsetsM": deepcopy(TOOL_TCP_OFFSETS_M),
            "toolProfiles": deepcopy(DEFAULT_TOOL_PROFILES),
            "updatedAt": None,
        }

    def _load(self) -> None:
        if not self.path.exists():
            return
        try:
            raw = json.loads(self.path.read_text())
        except (OSError, ValueError):
            return
        if not isinstance(raw, dict):
            return

        def records(key: str) -> List[Dict[str, Any]]:
            value = raw.get(key)
            return [item for item in value if isinstance(item, dict)] if isinstance(value, list) else []

        self.parts = {}
        for item in records("parts"):
            if not item.get("id"):
                continue
            try:
                part = self.normalized_part(item, item)
            except (TypeError, ValueError):
                continue
            self.parts[part["id"]] = part
        self.registered_parts = {}
        loaded_tag_ids = set()
        for item in records("registeredParts"):
            try:
                tag_id = int(item.get("tagId", -1))
            except (TypeError, ValueError):
                continue
            if item.get("partId") and 10 <= tag_id <= 25 and tag_id not in loaded_tag_ids:
                part_id = str(item["partId"])
                raw_size = item.get("size") if isinstance(item.get("size"), dict) else {}
                raw_offset = item.get("tagOffsetM") if isinstance(item.get("tagOffsetM"), dict) else {}
                self.registered_parts[part_id] = {
                    **item,
                    "partId": part_id,
                    "tagId": tag_id,
                    "label": str(item.get("label") or f"Tagged Part {tag_id}"),
                    "type": str(item.get("type") or "box"),
                    "size": {
                        "x": clamp(raw_size.get("x", 0.04), 0.008, 0.20),
                        "y": clamp(raw_size.get("y", 0.04), 0.008, 0.20),
                        "z": clamp(raw_size.get("z", 0.05), 0.008, 0.20),
                    },
                    "color": str(item.get("color") or "#8a63d2"),
                    "graspable": bool(item.get("graspable", True)),
                    "tagSizeM": 0.03,
                    "tagOffsetM": {
                        "x": clamp(raw_offset.get("x", 0.0), -0.20, 0.20),
                        "y": clamp(raw_offset.get("y", 0.0), -0.20, 0.20),
                    },
                    "yawOffsetDeg": _wrap_deg(item.get("yawOffsetDeg", 0.0) or 0.0),
                    "pickupProfiles": self._normalize_pickup_profiles(item.get("pickupProfiles")),
                }
                loaded_tag_ids.add(tag_id)
        self.bins = {}
        for item in records("bins"):
            if not item.get("id"):
                continue
            try:
                bin_obj = self.normalized_bin(item, item)
            except (TypeError, ValueError):
                continue
            self.bins[bin_obj["id"]] = bin_obj
        for bin_obj in self.bins.values():
            # Existing saved bins predate verification state and are assumed
            # to match the operator's established physical layout. Any future
            # AI/viewport relocation explicitly changes this to simulation_only.
            bin_obj.setdefault("positionStatus", "operator_verified")
            bin_obj.setdefault("positionSource", "legacy_or_operator")
        self.taught_points = {}
        for point in records("taughtPoints"):
            coords = point.get("firmwareFlangeCoordsMmDeg")
            angles = point.get("jointAnglesDeg")
            tcp = point.get("tcpPoseM")
            position = tcp.get("position") if isinstance(tcp, dict) else None
            values = (coords or []) + (angles or []) if isinstance(coords, list) and isinstance(angles, list) else []
            if (
                point.get("id")
                and len(coords or []) == 6
                and len(angles or []) == 6
                and len(values) == 12
                and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values)
                and isinstance(position, dict)
                and all(axis in position for axis in ("x", "y", "z"))
            ):
                self.taught_points[str(point["id"])] = point
        self.programs = {}
        for item in records("programs"):
            if not item.get("id"):
                continue
            try:
                program = self.normalized_program(item)
            except (TypeError, ValueError):
                continue
            self.programs[program["id"]] = program
        loaded_calibration = raw.get("calibration") if isinstance(raw.get("calibration"), dict) else {}
        defaults = self._default_calibration()
        loaded_fiducials = loaded_calibration.get("fiducials")
        if not isinstance(loaded_fiducials, dict):
            loaded_fiducials = {}
        self.calibration = _json_safe({
            **defaults,
            **loaded_calibration,
            "fiducials": {**defaults["fiducials"], **loaded_fiducials},
        })
        if not self.registered_parts:
            for legacy in (self.calibration.get("fiducials") or {}).get("objectTags") or []:
                if not isinstance(legacy, dict):
                    continue
                try:
                    tag_id = int(legacy.get("tagId", legacy.get("id", -1)))
                except (TypeError, ValueError):
                    continue
                if 10 <= tag_id <= 25:
                    part_id = str(legacy.get("partId") or legacy.get("objectId") or f"tag-part-{tag_id}")
                    self.registered_parts[part_id] = {
                        "partId": part_id, "tagId": tag_id,
                        "label": str(legacy.get("label") or f"Tagged Part {tag_id}"),
                        "type": str(legacy.get("type") or legacy.get("class") or "box"),
                        "size": deepcopy(legacy.get("size") or {"x": 0.04, "y": 0.04, "z": 0.05}),
                        "color": str(legacy.get("color") or "#8a63d2"), "graspable": bool(legacy.get("graspable", True)),
                        "tagSizeM": float(legacy.get("tagSizeM") or 0.03),
                        "tagOffsetM": deepcopy(legacy.get("tagOffsetM") or legacy.get("centerOffsetM") or {"x": 0.0, "y": 0.0}),
                        "yawOffsetDeg": float(legacy.get("yawOffsetDeg") or 0.0), "lastSeenAt": None,
                        "pickupProfiles": self._normalize_pickup_profiles(legacy.get("pickupProfiles")),
                    }
            self.calibration["fiducials"]["objectTags"] = []
        # Legacy camera/contour detections were transient observations, not
        # user-authored inventory. Do not resurrect them after migrating to
        # deterministic AprilTag tracking; manual/virtual parts are preserved.
        self.parts = {
            part_id: part for part_id, part in self.parts.items()
            if part.get("source") != "camera"
        }
        camera_defaults = self._default_camera()
        loaded_camera = raw.get("camera") if isinstance(raw.get("camera"), dict) else {}
        obsolete_camera_keys = {
            "classifier", "preferredName", "preferredIndex", "allowFallbackCameras", "label",
        }
        loaded_camera = {
            key: value for key, value in loaded_camera.items()
            if key not in obsolete_camera_keys
        }
        loaded_localization = loaded_camera.get("localization")
        if not isinstance(loaded_localization, dict):
            loaded_localization = {}
        loaded_workspace_bounds = loaded_camera.get("workspaceBounds")
        if not isinstance(loaded_workspace_bounds, dict):
            loaded_workspace_bounds = {}
        self.camera = {
            **camera_defaults,
            **loaded_camera,
            "localization": {**camera_defaults["localization"], **loaded_localization},
            "workspaceBounds": {**camera_defaults["workspaceBounds"], **loaded_workspace_bounds},
        }
        loaded_coordinate_planner = raw.get("coordinatePlanner") if isinstance(raw.get("coordinatePlanner"), dict) else {}
        self.coordinate_planner = {
            **self._default_coordinate_planner(),
            **loaded_coordinate_planner,
        }
        loaded_tool_profiles = loaded_coordinate_planner.get("toolProfiles") or {}
        if not isinstance(loaded_tool_profiles, dict):
            loaded_tool_profiles = {}
        self.coordinate_planner["toolProfiles"] = {
            tool_id: {
                **deepcopy(default_profile),
                **deepcopy(loaded_tool_profiles.get(tool_id) if isinstance(loaded_tool_profiles.get(tool_id), dict) else {}),
                "tcpCorrectionLocalM": {
                    **deepcopy(default_profile["tcpCorrectionLocalM"]),
                    **deepcopy(
                        loaded_tool_profiles.get(tool_id, {}).get("tcpCorrectionLocalM")
                        if isinstance(loaded_tool_profiles.get(tool_id), dict)
                        and isinstance(loaded_tool_profiles.get(tool_id, {}).get("tcpCorrectionLocalM"), dict)
                        else {}
                    ),
                },
                "geometry": {
                    **deepcopy(default_profile.get("geometry") or {}),
                    **deepcopy(
                        loaded_tool_profiles.get(tool_id, {}).get("geometry")
                        if isinstance(loaded_tool_profiles.get(tool_id), dict)
                        and isinstance(loaded_tool_profiles.get(tool_id, {}).get("geometry"), dict)
                        else {}
                    ),
                },
                **({
                    "hardware": {
                        **deepcopy(default_profile.get("hardware") or {}),
                        **deepcopy(
                            loaded_tool_profiles.get(tool_id, {}).get("hardware")
                            if isinstance(loaded_tool_profiles.get(tool_id), dict)
                            and isinstance(loaded_tool_profiles.get(tool_id, {}).get("hardware"), dict)
                            else {}
                        ),
                    }
                } if default_profile.get("hardware") else {}),
            }
            for tool_id, default_profile in DEFAULT_TOOL_PROFILES.items()
        }
        # Old builds exposed mutually exclusive flange-lift and TCP-offset
        # modes. New plans always use the modeled flange<->TCP transform, so
        # discard those obsolete runtime switches while loading old files.
        self.coordinate_planner.pop("toolOffsetMode", None)
        self.coordinate_planner.pop("legacyToolOffsetMode", None)
        self.coordinate_planner.pop("toolVerticalLiftM", None)
        self.end_effector = self._normalize_end_effector(raw.get("endEffector"))
        try:
            self.version = max(0, int(raw.get("version") or 0))
        except (TypeError, ValueError):
            self.version = 0
        try:
            self._counter = max(1, int(raw.get("counter") or 1))
        except (TypeError, ValueError):
            self._counter = 1

    def _save_locked(self) -> None:
        self.version += 1
        self.updated_at = time.time()
        payload = {
            "version": self.version,
            "counter": self._counter,
            "parts": [part for part in self.parts.values() if part.get("trackingMode") != "apriltag"],
            "registeredParts": list(self.registered_parts.values()),
            "bins": list(self.bins.values()),
            "taughtPoints": list(self.taught_points.values()),
            "programs": list(self.programs.values()),
            "calibration": self.calibration,
            "camera": self.camera,
            "coordinatePlanner": self.coordinate_planner,
            "endEffector": self.end_effector,
        }
        self.data_dir.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(_json_safe(payload), indent=1, allow_nan=False))
        os.replace(tmp, self.path)

    def _next_id(self, prefix: str) -> str:
        value = f"{prefix}-{self._counter}"
        self._counter += 1
        return value

    @staticmethod
    def _normalize_end_effector(value: Any) -> str:
        key = str(value or "adaptive_gripper")
        return key if key in END_EFFECTORS else "adaptive_gripper"

    @staticmethod
    def _normalize_pickup_profiles(raw: Any, existing: Any = None) -> Dict[str, Any]:
        supplied = raw if isinstance(raw, dict) else {}
        prior = existing if isinstance(existing, dict) else {}
        profiles = deepcopy(DEFAULT_PICKUP_PROFILES)
        for tool_id in END_EFFECTORS:
            source = {**(prior.get(tool_id) or {}), **(supplied.get(tool_id) or {})}
            offset = source.get("offsetLocalM") or profiles[tool_id]["offsetLocalM"]
            profiles[tool_id].update(source)
            profiles[tool_id]["offsetLocalM"] = {
                "x": clamp(offset.get("x", 0.0), -0.10, 0.10),
                "y": clamp(offset.get("y", 0.0), -0.10, 0.10),
                "z": clamp(offset.get("z", 0.0), -0.05, 0.05),
            }
        adaptive = profiles["adaptive_gripper"]
        adaptive["maximumTiltDeg"] = clamp(adaptive.get("maximumTiltDeg", 10.0), 0.0, 10.0)
        if adaptive.get("jawYawOverrideDeg") is not None:
            adaptive["jawYawOverrideDeg"] = _wrap_deg(float(adaptive["jawYawOverrideDeg"]))
            adaptive["jawYawMode"] = "manual"
        else:
            adaptive["jawYawMode"] = "automatic_narrow_side"
        suction = profiles["suction_gripper"]
        suction["contactPreloadM"] = clamp(suction.get("contactPreloadM", 0.002), 0.0, 0.008)
        suction["yawMode"] = "minimum_joint_travel"
        return profiles

    # --------------------------------------------------------------- parts

    def normalized_part(self, body: Dict[str, Any], existing: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        base = dict(existing or {})
        position = body.get("position") or base.get("position") or {}
        size = body.get("size") or base.get("size") or {}
        part_id = str(body.get("id") or base.get("id") or self._next_id("part"))
        part = {
            "id": part_id,
            "kind": "part",
            "label": str(body.get("label") or base.get("label") or part_id),
            "type": str(body.get("type") or base.get("type") or "box"),
            "position": {
                "x": clamp(position.get("x", 0.16), -SCENE_BOUND_M, SCENE_BOUND_M),
                "y": clamp(position.get("y", 0.10), -SCENE_BOUND_M, SCENE_BOUND_M),
                "z": clamp(position.get("z", 0.025), 0.0, 0.35),
            },
            "size": {
                "x": clamp(size.get("x", 0.05), 0.008, 0.20),
                "y": clamp(size.get("y", 0.05), 0.008, 0.20),
                "z": clamp(size.get("z", 0.05), 0.008, 0.20),
            },
            "orientationDeg": _wrap_deg(body.get("orientationDeg", base.get("orientationDeg", 0.0)) or 0.0),
            "color": str(body.get("color") or base.get("color") or "#2f80ed"),
            "graspable": bool(body.get("graspable", base.get("graspable", True))),
            "confidence": clamp(body.get("confidence", base.get("confidence", 1.0)), 0.0, 1.0),
            "source": str(body.get("source") or base.get("source") or "manual"),
            "updatedAt": time.time(),
            "pickupProfiles": self._normalize_pickup_profiles(
                body.get("pickupProfiles"), base.get("pickupProfiles")
            ),
        }
        tracking_mode = body.get("trackingMode", base.get("trackingMode"))
        if tracking_mode in ("virtual", "apriltag"):
            part["trackingMode"] = tracking_mode
        return part

    def upsert_part(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            existing = self.parts.get(str(body.get("id") or ""))
            part = self.normalized_part(body, existing)
            definition = self.registered_parts.get(part["id"])
            if definition:
                definition.update({
                    "label": part["label"], "type": part["type"], "size": deepcopy(part["size"]),
                    "color": part["color"], "graspable": part["graspable"], "updatedAt": time.time(),
                    "pickupProfiles": deepcopy(part["pickupProfiles"]),
                })
                # Editing a hidden registration must not fabricate a live pose.
                # Only update the live object when its tag is currently visible.
                if existing and existing.get("trackingMode") == "apriltag":
                    part.update({"trackingMode": "apriltag", "tagId": definition["tagId"], "source": "camera"})
                    self.parts[part["id"]] = part
                self._save_locked()
                return {**self.snapshot_locked(), "registeredPart": deepcopy(definition), "part": deepcopy(self.parts.get(part["id"]))}
            self.parts[part["id"]] = part
            self._save_locked()
            return {**self.snapshot_locked(), "part": part}

    def delete_part(self, part_id: str) -> Dict[str, Any]:
        with self.lock:
            self.parts.pop(str(part_id), None)
            self.registered_parts.pop(str(part_id), None)
            self.tag_last_seen.pop(str(part_id), None)
            self._save_locked()
            return self.snapshot_locked()

    def bind_tagged_part(self, body: Dict[str, Any]) -> Dict[str, Any]:
        tag_id = int(body.get("tagId", -1))
        if tag_id < 10 or tag_id > 25:
            return {"ok": False, "error": "Object AprilTag ID must be between 10 and 25."}
        with self.lock:
            requested_part_id = str(body.get("partId") or body.get("id") or "")
            part_id = requested_part_id or self._next_id("part")
            conflict = next((item for item in self.registered_parts.values() if int(item.get("tagId", -1)) == tag_id and item.get("partId") != part_id), None)
            if conflict and not body.get("reassign"):
                return {"ok": False, "error": f"Tag {tag_id} is already assigned to {conflict.get('label') or conflict['partId']}.", "requiresReassign": True, "conflictingPartId": conflict["partId"]}
            if conflict:
                self.registered_parts.pop(conflict["partId"], None)
                old = self.parts.get(conflict["partId"])
                if old:
                    old.update({"trackingMode": "virtual", "source": "manual"})
                    old.pop("tagId", None)
            existing_definition = self.registered_parts.get(part_id) or {}
            existing_part = self.parts.get(part_id) or {}
            raw_size = body.get("size") or existing_definition.get("size") or existing_part.get("size") or {}
            size = {
                "x": clamp(raw_size.get("x", 0.04), 0.008, 0.20),
                "y": clamp(raw_size.get("y", 0.04), 0.008, 0.20),
                "z": clamp(raw_size.get("z", 0.05), 0.008, 0.20),
            }
            raw_offset = body.get("tagOffsetM") or existing_definition.get("tagOffsetM") or {}
            definition = {
                "partId": part_id, "tagId": tag_id,
                "label": str(body.get("label") or existing_definition.get("label") or existing_part.get("label") or f"Tagged Part {tag_id}"),
                "type": str(body.get("type") or existing_definition.get("type") or existing_part.get("type") or "box"),
                "size": size, "color": str(body.get("color") or existing_definition.get("color") or existing_part.get("color") or "#8a63d2"),
                "graspable": bool(body.get("graspable", existing_definition.get("graspable", existing_part.get("graspable", True)))),
                "tagSizeM": 0.03,
                "tagOffsetM": {"x": clamp(raw_offset.get("x", 0.0), -0.20, 0.20), "y": clamp(raw_offset.get("y", 0.0), -0.20, 0.20)},
                "yawOffsetDeg": _wrap_deg(body.get("yawOffsetDeg", existing_definition.get("yawOffsetDeg", 0.0)) or 0.0),
                "pickupProfiles": self._normalize_pickup_profiles(
                    body.get("pickupProfiles"), existing_definition.get("pickupProfiles")
                ),
                "lastSeenAt": existing_definition.get("lastSeenAt"), "updatedAt": time.time(),
            }
            self.registered_parts[part_id] = definition
            # A binding is live only after the camera observes the selected tag.
            self.parts.pop(part_id, None)
            self.tag_last_seen.pop(part_id, None)
            self._save_locked()
            return {**self.snapshot_locked(), "registeredPart": deepcopy(definition)}

    def unbind_tagged_part(self, part_id: str) -> Dict[str, Any]:
        with self.lock:
            definition = self.registered_parts.pop(str(part_id), None)
            if not definition:
                return {"ok": False, "error": "Tagged part registration was not found."}
            current = self.parts.get(str(part_id))
            position = deepcopy((current or {}).get("position") or {"x": 0.16, "y": 0.08, "z": float(definition["size"]["z"]) / 2.0})
            part = self.normalized_part({
                "id": part_id, "label": definition["label"], "type": definition["type"],
                "position": position, "size": definition["size"], "color": definition["color"],
                "graspable": definition["graspable"], "source": "manual",
                "orientationDeg": (current or {}).get("orientationDeg", 0.0),
                "pickupProfiles": definition.get("pickupProfiles"),
            }, current)
            part["trackingMode"] = "virtual"
            self.parts[str(part_id)] = part
            self.tag_last_seen.pop(str(part_id), None)
            self._save_locked()
            return {**self.snapshot_locked(), "part": part}

    def ingest_tag_tracks(self, detections: List[Dict[str, Any]], timestamp: Optional[float] = None, valid: bool = True) -> Dict[str, Any]:
        """Update live tagged objects without persisting per-frame camera state."""
        now = float(timestamp or time.time())
        by_id = {str(item.get("id") or ""): item for item in detections if item.get("localizationSource") == "object_tag"}
        with self.lock:
            pose_changed = False
            membership_changed = False
            for part_id, definition in self.registered_parts.items():
                detection = by_id.get(part_id) if valid else None
                if detection:
                    was_visible = part_id in self.parts and self.parts[part_id].get("trackingMode") == "apriltag"
                    part = self.normalized_part({
                        "id": part_id, "label": definition["label"], "type": definition["type"],
                        "position": detection.get("position"), "size": definition["size"],
                        "orientationDeg": detection.get("orientationDeg", 0.0), "color": definition["color"],
                        "graspable": definition["graspable"], "confidence": detection.get("confidence", 0.99), "source": "camera",
                        "pickupProfiles": definition.get("pickupProfiles"),
                    }, self.parts.get(part_id))
                    part.update({
                        "trackingMode": "apriltag", "tagId": definition["tagId"], "lastSeenAt": now,
                        "poseQuality": detection.get("poseQuality") or detection.get("calibrationQuality"),
                        "localizationSource": "object_tag", "bboxPx": detection.get("bboxPx"), "stale": False,
                        "reservedByPlan": (self.parts.get(part_id) or {}).get("reservedByPlan"),
                        "reservationCreatedAt": (self.parts.get(part_id) or {}).get("reservationCreatedAt"),
                    })
                    self.parts[part_id] = part
                    self.tag_last_seen[part_id] = now
                    definition["lastSeenAt"] = now
                    pose_changed = True
                    membership_changed = membership_changed or not was_visible
                elif part_id in self.parts and self.parts[part_id].get("trackingMode") == "apriltag":
                    if now - float(self.tag_last_seen.get(part_id) or 0.0) >= 1.0:
                        del self.parts[part_id]
                        pose_changed = True
                        membership_changed = True
            if pose_changed:
                self.tag_track_revision += 1
                self.updated_at = now
            # Full scene snapshots are versioned only when membership changes;
            # high-rate poses travel through the lightweight track endpoint.
            if membership_changed:
                self.version += 1
            return self.tag_tracks_locked()

    def tag_tracks_locked(self) -> Dict[str, Any]:
        visible = [deepcopy(part) for part in self.parts.values() if part.get("trackingMode") == "apriltag"]
        visible_ids = {part["id"] for part in visible}
        return {
            "ok": True, "revision": self.tag_track_revision, "timestamp": time.time(),
            "parts": visible, "removedIds": sorted(set(self.registered_parts) - visible_ids),
        }

    def tag_tracks(self, since: Optional[int] = None) -> Dict[str, Any]:
        with self.lock:
            if since is not None and int(since) >= self.tag_track_revision:
                return {
                    "ok": True, "revision": self.tag_track_revision, "timestamp": time.time(),
                    "parts": [], "removedIds": [], "unchanged": True,
                }
            return self.tag_tracks_locked()

    # ---------------------------------------------------------------- bins

    def normalized_bin(self, body: Dict[str, Any], existing: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        base = dict(existing or {})
        position = body.get("position") or base.get("position") or {}
        outer = body.get("outer") or base.get("outer") or {}
        bin_id = str(body.get("id") or base.get("id") or self._next_id("bin"))
        wall = clamp(body.get("wallThickness", base.get("wallThickness", 0.008)), 0.003, 0.03)
        # Bins are flat drop rectangles by default; outer.z only matters if a
        # walled bin is configured (planner uses it for carry clearance).
        outer_x = clamp(outer.get("x", 0.14), 0.04, 0.4)
        outer_y = clamp(outer.get("y", 0.14), 0.04, 0.4)
        outer_z = clamp(outer.get("z", 0.02), 0.01, 0.2)
        # Interior boundary must stay a usable opening.
        wall = min(wall, (min(outer_x, outer_y) - 0.02) / 2.0)
        return {
            "id": bin_id,
            "kind": "bin",
            "label": str(body.get("label") or base.get("label") or bin_id),
            "position": {
                "x": clamp(position.get("x", 0.20), -SCENE_BOUND_M, SCENE_BOUND_M),
                "y": clamp(position.get("y", -0.10), -SCENE_BOUND_M, SCENE_BOUND_M),
                "z": clamp(position.get("z", 0.0), 0.0, 0.2),
            },
            "outer": {"x": outer_x, "y": outer_y, "z": outer_z},
            "wallThickness": wall,
            "orientationDeg": _wrap_deg(body.get("orientationDeg", base.get("orientationDeg", 0.0)) or 0.0),
            "color": str(body.get("color") or base.get("color") or "#f59e0b"),
            "positionStatus": str(
                body.get("positionStatus")
                or base.get("positionStatus")
                or "operator_verified"
            ),
            "positionSource": str(
                body.get("positionSource")
                or base.get("positionSource")
                or "operator"
            ),
            "updatedAt": time.time(),
        }

    @staticmethod
    def bin_geometry(bin_obj: Dict[str, Any]) -> Dict[str, Any]:
        """Derived geometry: interior boundary, drop pose, wall top height."""
        wall = float(bin_obj["wallThickness"])
        outer = bin_obj["outer"]
        position = bin_obj["position"]
        interior_x = max(0.01, float(outer["x"]) - 2.0 * wall)
        interior_y = max(0.01, float(outer["y"]) - 2.0 * wall)
        floor_z = float(position["z"]) + wall
        wall_top_z = float(position["z"]) + float(outer["z"])
        return {
            "interior": {"x": round(interior_x, 4), "y": round(interior_y, 4)},
            "floorZ": round(floor_z, 4),
            "wallTopZ": round(wall_top_z, 4),
            "dropCenter": {
                "x": round(float(position["x"]), 4),
                "y": round(float(position["y"]), 4),
                "z": round(floor_z, 4),
            },
        }

    @staticmethod
    def reachable_bin_drop_xy(
        bin_obj: Dict[str, Any], geometry: Dict[str, Any], part_size: Dict[str, Any]
    ) -> Dict[str, float]:
        """Closest-to-base drop center that keeps the complete part in the bin."""
        center_x = float(bin_obj["position"]["x"])
        center_y = float(bin_obj["position"]["y"])
        yaw = math.radians(float(bin_obj.get("orientationDeg", 0.0)))
        c, s = math.cos(yaw), math.sin(yaw)
        # Base origin expressed in the bin's local XY frame.
        desired_local_x = c * (-center_x) + s * (-center_y)
        desired_local_y = -s * (-center_x) + c * (-center_y)
        limit_x = max(
            0.0,
            float(geometry["interior"]["x"]) / 2.0
            - float(part_size["x"]) / 2.0
            - BIN_DROP_WALL_CLEARANCE_M,
        )
        limit_y = max(
            0.0,
            float(geometry["interior"]["y"]) / 2.0
            - float(part_size["y"]) / 2.0
            - BIN_DROP_WALL_CLEARANCE_M,
        )
        local_x = max(-limit_x, min(limit_x, desired_local_x))
        local_y = max(-limit_y, min(limit_y, desired_local_y))
        return {
            "x": round(center_x + c * local_x - s * local_y, 4),
            "y": round(center_y + s * local_x + c * local_y, 4),
        }

    def upsert_bin(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            existing = self.bins.get(str(body.get("id") or ""))
            bin_obj = self.normalized_bin(body, existing)
            self.bins[bin_obj["id"]] = bin_obj
            self._save_locked()
            return {**self.snapshot_locked(), "bin": self._bin_with_geometry(bin_obj)}

    def delete_bin(self, bin_id: str) -> Dict[str, Any]:
        with self.lock:
            self.bins.pop(str(bin_id), None)
            self._save_locked()
            return self.snapshot_locked()

    def confirm_bin_position(self, bin_id: str) -> Dict[str, Any]:
        with self.lock:
            bin_obj = self.bins.get(str(bin_id))
            if bin_obj is None:
                return {"ok": False, "error": "Bin was not found."}
            bin_obj["positionStatus"] = "operator_verified"
            bin_obj["positionSource"] = "operator_confirmation"
            bin_obj["positionVerifiedAt"] = time.time()
            bin_obj["updatedAt"] = time.time()
            self._save_locked()
            return {**self.snapshot_locked(), "bin": self._bin_with_geometry(bin_obj)}

    def _bin_with_geometry(self, bin_obj: Dict[str, Any]) -> Dict[str, Any]:
        return {**bin_obj, "geometry": self.bin_geometry(bin_obj)}

    # ------------------------------------------------------ taught points

    def tool_calibration_fingerprint(self, tool_id: Optional[str] = None) -> str:
        selected = self._normalize_end_effector(tool_id or self.end_effector)
        profiles = (self.coordinate_planner or {}).get("toolProfiles") or DEFAULT_TOOL_PROFILES
        offsets = (self.coordinate_planner or {}).get("toolOffsetsM") or TOOL_TCP_OFFSETS_M
        material = {
            "tool": selected,
            "profile": profiles.get(selected) or DEFAULT_TOOL_PROFILES[selected],
            "offset": offsets.get(selected) or TOOL_TCP_OFFSETS_M[selected],
        }
        encoded = json.dumps(_json_safe(material), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(encoded.encode("utf-8")).hexdigest()[:16]

    def normalized_taught_point(
        self, body: Dict[str, Any], existing: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        base = dict(existing or {})
        point_id = str(body.get("id") or base.get("id") or self._next_id("point"))
        tool_id = self._normalize_end_effector(
            body.get("endEffector") or base.get("endEffector") or self.end_effector
        )
        tcp = body.get("tcpPoseM") or base.get("tcpPoseM") or {}
        tcp_position = tcp.get("position") or {}
        tcp_rpy = tcp.get("rpyDeg") or {}
        flange = body.get("flangePoseM") or base.get("flangePoseM") or {}
        flange_position = flange.get("position") or {}
        flange_rpy = flange.get("rpyDeg") or {}
        raw_coords = body.get("firmwareFlangeCoordsMmDeg") or base.get("firmwareFlangeCoordsMmDeg") or []
        raw_angles = body.get("jointAnglesDeg") or base.get("jointAnglesDeg") or []
        if len(raw_coords) != 6 or len(raw_angles) != 6:
            raise ValueError("A taught point requires six flange coordinates and six joint angles.")
        coords = [float(value) for value in raw_coords]
        angles = [float(value) for value in raw_angles]
        if not all(math.isfinite(value) for value in coords + angles):
            raise ValueError("Taught-point coordinates and angles must be finite.")
        coordinate_errors = validate_coordinate_bounds(
            coords, "taught_point", allow_missing_rpy=False
        )
        if coordinate_errors:
            raise ValueError(
                coordinate_errors[0].get("message")
                or coordinate_errors[0].get("error")
                or "Taught-point coordinates are outside the robot envelope."
            )
        for joint, value in enumerate(angles, 1):
            low, high = JOINT_LIMITS_DEG[joint]
            if value < low or value > high:
                raise ValueError(
                    f"Taught-point J{joint} angle {value:.2f} is outside {low:.1f} to {high:.1f} degrees."
                )
        supplied_fingerprint = body.get("toolCalibrationFingerprint") or base.get("toolCalibrationFingerprint")
        current_fingerprint = self.tool_calibration_fingerprint(tool_id)
        if supplied_fingerprint and str(supplied_fingerprint) != current_fingerprint:
            raise ValueError("Tool calibration changed before this point was saved; capture it again.")
        expected_flange_position = tuple(value / 1000.0 for value in coords[:3])
        expected_flange_rotation = rotation_from_rpy_deg(coords[3:6])
        if flange_position:
            supplied_flange_position = tuple(float(flange_position.get(axis, 0.0)) for axis in ("x", "y", "z"))
            supplied_flange_rpy = [float(flange_rpy.get(axis, 0.0)) for axis in ("rx", "ry", "rz")]
            _, flange_orientation_error = pose_residual(
                supplied_flange_position,
                rotation_from_rpy_deg(supplied_flange_rpy),
                expected_flange_position,
                expected_flange_rotation,
            )
            if math.dist(supplied_flange_position, expected_flange_position) > 0.001 or flange_orientation_error > math.radians(1.0):
                raise ValueError("Saved flange pose is inconsistent with the measured firmware coordinates.")
        profile = ((self.coordinate_planner or {}).get("toolProfiles") or {}).get(tool_id) or {}
        correction = profile.get("tcpCorrectionLocalM") or {}
        correction_local = tuple(float(correction.get(axis, 0.0)) for axis in ("x", "y", "z"))
        suction_distance = float((profile.get("geometry") or {}).get("flangeToContactM", 0.072))
        expected_tcp_position, expected_tcp_rotation = tcp_from_flange(
            expected_flange_position, expected_flange_rotation,
            tool_id, correction_local, suction_distance,
        )
        expected_tcp_rpy = rpy_deg_from_rotation(expected_tcp_rotation)
        if tcp_position:
            supplied_tcp_position = tuple(float(tcp_position.get(axis, 0.0)) for axis in ("x", "y", "z"))
            supplied_tcp_rpy = [float(tcp_rpy.get(axis, 0.0)) for axis in ("rx", "ry", "rz")]
            _, tcp_orientation_error = pose_residual(
                supplied_tcp_position,
                rotation_from_rpy_deg(supplied_tcp_rpy),
                expected_tcp_position,
                expected_tcp_rotation,
            )
            if math.dist(supplied_tcp_position, expected_tcp_position) > 0.001 or tcp_orientation_error > math.radians(1.0):
                raise ValueError("Saved TCP pose is inconsistent with the measured flange pose and active tool.")
        uses = body.get("uses", base.get("uses", ["waypoint", "destination"]))
        uses = [value for value in (uses if isinstance(uses, list) else []) if value in ("waypoint", "destination")]
        if not uses:
            uses = ["waypoint", "destination"]
        support_z = body.get("supportSurfaceZ", base.get("supportSurfaceZ"))
        normalized_support_z = None
        if support_z is not None:
            try:
                normalized_support_z = float(support_z)
            except (TypeError, ValueError):
                raise ValueError("Support-surface Z must be a finite number in meters.")
            if not math.isfinite(normalized_support_z):
                raise ValueError("Support-surface Z must be a finite number in meters.")
            if normalized_support_z < 0.0 or normalized_support_z > 0.30:
                raise ValueError("Support-surface Z must be between 0.0 and 0.30 meters.")
        return {
            "id": point_id,
            "kind": "point",
            "label": str(body.get("label") or base.get("label") or point_id),
            "frame": "robot_base_meters",
            "tcpPoseM": {
                "position": {
                    "x": clamp(tcp_position.get("x", expected_tcp_position[0]), -SCENE_BOUND_M, SCENE_BOUND_M),
                    "y": clamp(tcp_position.get("y", expected_tcp_position[1]), -SCENE_BOUND_M, SCENE_BOUND_M),
                    "z": clamp(tcp_position.get("z", expected_tcp_position[2]), -0.07, 0.45),
                },
                "rpyDeg": {
                    "rx": _wrap_deg(tcp_rpy.get("rx", expected_tcp_rpy[0])),
                    "ry": _wrap_deg(tcp_rpy.get("ry", expected_tcp_rpy[1])),
                    "rz": _wrap_deg(tcp_rpy.get("rz", expected_tcp_rpy[2])),
                },
            },
            "flangePoseM": {
                "position": {
                    "x": float(flange_position.get("x", coords[0] / 1000.0)),
                    "y": float(flange_position.get("y", coords[1] / 1000.0)),
                    "z": float(flange_position.get("z", coords[2] / 1000.0)),
                },
                "rpyDeg": {
                    "rx": _wrap_deg(flange_rpy.get("rx", coords[3])),
                    "ry": _wrap_deg(flange_rpy.get("ry", coords[4])),
                    "rz": _wrap_deg(flange_rpy.get("rz", coords[5])),
                },
            },
            "firmwareFlangeCoordsMmDeg": [round(value, 6) for value in coords],
            "jointAnglesDeg": [round(value, 6) for value in angles],
            "endEffector": tool_id,
            "toolCalibrationFingerprint": str(
                body.get("toolCalibrationFingerprint")
                or base.get("toolCalibrationFingerprint")
                or current_fingerprint
            ),
            "supportSurfaceZ": normalized_support_z,
            "uses": sorted(set(uses)),
            "capturedAt": float(body.get("capturedAt") or base.get("capturedAt") or time.time()),
            "updatedAt": time.time(),
        }

    def save_taught_point(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            try:
                existing = self.taught_points.get(str(body.get("id") or ""))
                point = self.normalized_taught_point(body, existing)
            except (TypeError, ValueError) as exc:
                return {"ok": False, "error": str(exc)}
            self.taught_points[point["id"]] = point
            self._save_locked()
            return {**self.snapshot_locked(), "point": deepcopy(point)}

    def delete_taught_point(self, point_id: str) -> Dict[str, Any]:
        with self.lock:
            self.taught_points.pop(str(point_id), None)
            self._save_locked()
            return self.snapshot_locked()

    # ---------------------------------------------------- spatial context

    def workspace_regions_locked(self) -> Dict[str, Any]:
        fiducials = (self.calibration or {}).get("fiducials") or {}
        if not fiducials.get("baselineHomography"):
            return {
                "available": False,
                "reason": "calibrated_workspace_bounds_unavailable",
                "frame": "robot_base_meters",
            }
        markers = [
            marker for marker in (fiducials.get("referenceMarkers") or [])
            if isinstance(marker, dict) and isinstance(marker.get("center"), dict)
        ]
        if len({int(marker.get("id", -1)) for marker in markers}) < 3:
            return {
                "available": False,
                "reason": "calibrated_workspace_bounds_unavailable",
                "frame": "robot_base_meters",
            }
        xs = [float(marker["center"]["x"]) for marker in markers]
        ys = [float(marker["center"]["y"]) for marker in markers]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        if x_max - x_min < 0.10 or y_max - y_min < 0.10:
            return {
                "available": False,
                "reason": "calibrated_workspace_bounds_degenerate",
                "frame": "robot_base_meters",
            }
        x_third = (x_max - x_min) / 3.0
        y_third = (y_max - y_min) / 3.0
        regions = {
            "right": {"xMin": x_min, "xMax": x_max, "yMin": y_min, "yMax": y_min + y_third},
            "left": {"xMin": x_min, "xMax": x_max, "yMin": y_max - y_third, "yMax": y_max},
            "front": {"xMin": x_max - x_third, "xMax": x_max, "yMin": y_min, "yMax": y_max},
            "back": {"xMin": x_min, "xMax": x_min + x_third, "yMin": y_min, "yMax": y_max},
            "center": {
                "xMin": x_min + x_third,
                "xMax": x_max - x_third,
                "yMin": y_min + y_third,
                "yMax": y_max - y_third,
            },
        }
        return {
            "available": True,
            "frame": "robot_base_meters",
            "coordinateConvention": {
                "front": "+X", "back": "-X", "left": "+Y", "right": "-Y", "up": "+Z",
            },
            "bounds": {"xMin": x_min, "xMax": x_max, "yMin": y_min, "yMax": y_max},
            "regions": regions,
            "source": "workspace_reference_marker_centers",
            "calibrationLocked": bool(fiducials.get("baselineHomography")),
        }

    @staticmethod
    def _rotated_half_extents(size_x: float, size_y: float, yaw_deg: float) -> Tuple[float, float]:
        yaw = math.radians(float(yaw_deg))
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        return (
            c * float(size_x) / 2.0 + s * float(size_y) / 2.0,
            s * float(size_x) / 2.0 + c * float(size_y) / 2.0,
        )

    @classmethod
    def _entity_footprint(cls, entity: Dict[str, Any]) -> Tuple[float, float]:
        size = entity.get("size") or entity.get("outer") or {}
        return cls._rotated_half_extents(
            float(size.get("x", 0.04)), float(size.get("y", 0.04)),
            float(entity.get("orientationDeg") or 0.0),
        )

    @staticmethod
    def _rectangles_overlap(
        a_x: float, a_y: float, a_hx: float, a_hy: float,
        b_x: float, b_y: float, b_hx: float, b_hy: float,
        margin: float = SPATIAL_PLACEMENT_MARGIN_M,
    ) -> bool:
        return (
            abs(float(a_x) - float(b_x)) < float(a_hx) + float(b_hx) + margin
            and abs(float(a_y) - float(b_y)) < float(a_hy) + float(b_hy) + margin
        )

    def _spatial_candidate_is_clear_locked(
        self,
        entity_kind: str,
        entity_id: str,
        x: float,
        y: float,
        half_x: float,
        half_y: float,
    ) -> Tuple[bool, Optional[str]]:
        if math.hypot(x, y) < BASE_TRANSFER_CLEARANCE_RADIUS_M + math.hypot(half_x, half_y):
            return False, "robot_base_exclusion"
        obstacles: List[Tuple[str, Dict[str, Any]]] = []
        obstacles.extend(("part", item) for item in self.parts.values())
        obstacles.extend(("bin", item) for item in self.bins.values())
        for kind, obstacle in obstacles:
            if kind == entity_kind and str(obstacle.get("id")) == str(entity_id):
                continue
            position = obstacle.get("position") or {}
            obstacle_hx, obstacle_hy = self._entity_footprint(obstacle)
            if self._rectangles_overlap(
                x, y, half_x, half_y,
                float(position.get("x", 0.0)), float(position.get("y", 0.0)),
                obstacle_hx, obstacle_hy,
            ):
                return False, f"occupied_by_{kind}:{obstacle.get('id')}"
        for marker in ((self.calibration or {}).get("fiducials") or {}).get("referenceMarkers") or []:
            center = marker.get("center") or {}
            marker_half = float(marker.get("sizeM") or 0.05) / 2.0
            if self._rectangles_overlap(
                x, y, half_x, half_y,
                float(center.get("x", 0.0)), float(center.get("y", 0.0)),
                marker_half, marker_half, margin=0.004,
            ):
                return False, f"workspace_marker:{marker.get('id')}"
        return True, None

    def _region_candidates_locked(
        self,
        entity_kind: str,
        entity: Dict[str, Any],
        region_name: str,
        limit: int = 40,
    ) -> Dict[str, Any]:
        workspace = self.workspace_regions_locked()
        if not workspace.get("available"):
            return {"ok": False, "error": workspace.get("reason"), "workspace": workspace}
        region = (workspace.get("regions") or {}).get(str(region_name).lower())
        if region is None:
            return {"ok": False, "error": f"Unknown workspace region '{region_name}'.", "workspace": workspace}
        half_x, half_y = self._entity_footprint(entity)
        x_min = float(region["xMin"]) + half_x + SPATIAL_PLACEMENT_MARGIN_M
        x_max = float(region["xMax"]) - half_x - SPATIAL_PLACEMENT_MARGIN_M
        y_min = float(region["yMin"]) + half_y + SPATIAL_PLACEMENT_MARGIN_M
        y_max = float(region["yMax"]) - half_y - SPATIAL_PLACEMENT_MARGIN_M
        if x_min > x_max or y_min > y_max:
            return {"ok": False, "error": f"{entity.get('label') or entity.get('id')} does not fit in the {region_name} region."}
        current = entity.get("position") or {}
        preferred_x = min(x_max, max(x_min, float(current.get("x", (x_min + x_max) / 2.0))))
        preferred_y = min(y_max, max(y_min, float(current.get("y", (y_min + y_max) / 2.0))))
        if region_name == "front":
            # Enter the requested third at its nearest edge. This minimizes
            # travel and avoids turning “front” into “furthest possible reach.”
            preferred_x = x_min
        elif region_name == "back":
            preferred_x = x_max
        elif region_name == "left":
            preferred_y = y_min
        elif region_name == "right":
            preferred_y = y_max
        xs = []
        value = x_min
        while value <= x_max + 1e-9:
            xs.append(value)
            value += SPATIAL_GRID_STEP_M
        ys = []
        value = y_min
        while value <= y_max + 1e-9:
            ys.append(value)
            value += SPATIAL_GRID_STEP_M
        xs.extend([x_max, preferred_x])
        ys.extend([y_max, preferred_y])
        candidates = []
        rejected: Dict[str, int] = {}
        seen = set()
        for x in xs:
            for y in ys:
                key = (round(x, 4), round(y, 4))
                if key in seen:
                    continue
                seen.add(key)
                clear, reason = self._spatial_candidate_is_clear_locked(
                    entity_kind, str(entity.get("id") or ""), x, y, half_x, half_y
                )
                if not clear:
                    rejected[reason or "occupied"] = rejected.get(reason or "occupied", 0) + 1
                    continue
                score = math.hypot(x - preferred_x, y - preferred_y)
                candidates.append({
                    "position": {"x": round(x, 4), "y": round(y, 4), "z": 0.0},
                    "score": round(score, 6),
                    "region": region_name,
                })
        candidates.sort(key=lambda item: item["score"])
        if not candidates:
            return {
                "ok": False,
                "error": f"No collision-free placement is available in the {region_name} region.",
                "rejected": rejected,
                "workspace": workspace,
            }
        return {
            "ok": True,
            "destinationKind": "region",
            "region": region_name,
            "candidates": candidates[:limit],
            "workspace": workspace,
            "coordinateReason": (
                f"Selected from calibrated {region_name} region; complete footprint stays inside the workspace "
                "and clear of the robot base, markers, parts, and bins."
            ),
        }

    def resolve_spatial_destination(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            entity_kind = str(body.get("entityKind") or "part").lower()
            entity_id = str(body.get("entityId") or body.get("objectId") or body.get("binId") or "")
            collection = self.parts if entity_kind == "part" else self.bins if entity_kind == "bin" else {}
            entity = collection.get(entity_id)
            if entity is None:
                return {"ok": False, "error": f"{entity_kind.title()} '{entity_id}' was not found."}
            destination = body.get("destination") or {}
            kind = str(destination.get("kind") or body.get("destinationKind") or "region").lower()
            if kind == "region":
                return {
                    **self._region_candidates_locked(
                        entity_kind, entity,
                        str(destination.get("region") or body.get("region") or "center").lower(),
                    ),
                    "entity": deepcopy(entity),
                }
            if kind == "relative":
                current = entity.get("position") or {}
                target = {
                    "x": float(current.get("x", 0.0)) + float(destination.get("dxM") or 0.0),
                    "y": float(current.get("y", 0.0)) + float(destination.get("dyM") or 0.0),
                    "z": float(destination.get("surfaceZ") or 0.0),
                }
                workspace = self.workspace_regions_locked()
                half_x, half_y = self._entity_footprint(entity)
                bounds = workspace.get("bounds") or {}
                if not workspace.get("available") or not (
                    float(bounds["xMin"]) + half_x <= target["x"] <= float(bounds["xMax"]) - half_x
                    and float(bounds["yMin"]) + half_y <= target["y"] <= float(bounds["yMax"]) - half_y
                ):
                    return {"ok": False, "error": "Relative destination falls outside calibrated workspace bounds."}
                clear, reason = self._spatial_candidate_is_clear_locked(entity_kind, entity_id, target["x"], target["y"], half_x, half_y)
                if not clear:
                    return {"ok": False, "error": f"Relative destination is blocked ({reason})."}
                return {
                    "ok": True, "entity": deepcopy(entity), "destinationKind": "relative",
                    "candidates": [{"position": target, "score": 0.0}],
                    "workspace": workspace,
                    "coordinateReason": "Applied the requested robot-frame relative offset and validated the footprint.",
                }
            if kind == "point":
                point = self.taught_points.get(str(destination.get("pointId") or body.get("pointId") or ""))
                if point is None or "destination" not in point.get("uses", []):
                    return {"ok": False, "error": "Destination taught point was not found or is not enabled for placement."}
                position = point["tcpPoseM"]["position"]
                target_x, target_y = float(position["x"]), float(position["y"])
                workspace = self.workspace_regions_locked()
                half_x, half_y = self._entity_footprint(entity)
                bounds = workspace.get("bounds") or {}
                if not workspace.get("available") or not (
                    float(bounds["xMin"]) + half_x + SPATIAL_PLACEMENT_MARGIN_M <= target_x <= float(bounds["xMax"]) - half_x - SPATIAL_PLACEMENT_MARGIN_M
                    and float(bounds["yMin"]) + half_y + SPATIAL_PLACEMENT_MARGIN_M <= target_y <= float(bounds["yMax"]) - half_y - SPATIAL_PLACEMENT_MARGIN_M
                ):
                    return {"ok": False, "error": "Taught destination does not keep the complete object footprint inside calibrated workspace bounds."}
                clear, reason = self._spatial_candidate_is_clear_locked(
                    entity_kind, entity_id, target_x, target_y, half_x, half_y
                )
                if not clear:
                    return {"ok": False, "error": f"Taught destination is blocked ({reason})."}
                return {
                    "ok": True, "entity": deepcopy(entity), "destinationKind": "point",
                    "point": deepcopy(point),
                    "candidates": [{"position": {
                        "x": target_x, "y": target_y,
                        "z": float(point.get("supportSurfaceZ") or 0.0),
                    }, "pointId": point["id"], "score": 0.0}],
                    "workspace": workspace,
                    "coordinateReason": f"Using taught destination point {point['label']} in robot-base coordinates.",
                }
            if kind == "bin" and entity_kind == "part":
                bin_obj = self.bins.get(str(destination.get("binId") or body.get("destinationId") or ""))
                if bin_obj is None:
                    return {"ok": False, "error": "Destination bin was not found."}
                geometry = self.bin_geometry(bin_obj)
                part_hx, part_hy = self._entity_footprint(entity)
                if (
                    2.0 * (part_hx + BIN_DROP_WALL_CLEARANCE_M) > float(geometry["interior"]["x"])
                    or 2.0 * (part_hy + BIN_DROP_WALL_CLEARANCE_M) > float(geometry["interior"]["y"])
                ):
                    return {"ok": False, "error": f"{entity.get('label')} does not fit completely inside {bin_obj.get('label')}."}
                return {
                    "ok": True, "entity": deepcopy(entity), "destinationKind": "bin",
                    "bin": deepcopy(bin_obj),
                    "candidates": [{"position": deepcopy(geometry["dropCenter"]), "binId": bin_obj["id"], "score": 0.0}],
                    "coordinateReason": f"Using {bin_obj['label']}'s configured interior drop region.",
                }
            if kind == "next_to":
                reference_kind = str(destination.get("referenceKind") or "bin")
                reference_id = str(destination.get("referenceId") or "")
                reference = (self.bins if reference_kind == "bin" else self.parts).get(reference_id)
                if reference is None:
                    return {"ok": False, "error": "Reference object for next-to placement was not found."}
                entity_hx, entity_hy = self._entity_footprint(entity)
                ref_hx, ref_hy = self._entity_footprint(reference)
                ref_position = reference.get("position") or {}
                side = str(destination.get("side") or "right").lower()
                offsets = {
                    "right": (0.0, -(entity_hy + ref_hy + SPATIAL_PLACEMENT_MARGIN_M)),
                    "left": (0.0, entity_hy + ref_hy + SPATIAL_PLACEMENT_MARGIN_M),
                    "front": (entity_hx + ref_hx + SPATIAL_PLACEMENT_MARGIN_M, 0.0),
                    "back": (-(entity_hx + ref_hx + SPATIAL_PLACEMENT_MARGIN_M), 0.0),
                }
                if side not in offsets:
                    return {"ok": False, "error": f"Unsupported next-to side '{side}'."}
                dx, dy = offsets[side]
                relative_body = {
                    "entityKind": entity_kind, "entityId": entity_id,
                    "destination": {
                        "kind": "relative",
                        "dxM": float(ref_position.get("x", 0.0)) + dx - float((entity.get("position") or {}).get("x", 0.0)),
                        "dyM": float(ref_position.get("y", 0.0)) + dy - float((entity.get("position") or {}).get("y", 0.0)),
                    },
                }
            else:
                return {"ok": False, "error": f"Unsupported spatial destination kind '{kind}'."}
        return self.resolve_spatial_destination(relative_body)

    def spatial_context(self) -> Dict[str, Any]:
        with self.lock:
            workspace = self.workspace_regions_locked()
            entities = []
            occupancy = []
            for part in self.parts.values():
                entities.append({
                    "kind": "part", "id": part["id"], "label": part.get("label"),
                    "position": deepcopy(part.get("position")), "size": deepcopy(part.get("size")),
                    "visible": True, "trackingMode": part.get("trackingMode"),
                })
                half_x, half_y = self._entity_footprint(part)
                occupancy.append({
                    "kind": "part", "id": part["id"], "center": deepcopy(part.get("position")),
                    "halfExtents": {"x": half_x, "y": half_y},
                })
            for bin_obj in self.bins.values():
                entities.append({
                    "kind": "bin", "id": bin_obj["id"], "label": bin_obj.get("label"),
                    "position": deepcopy(bin_obj.get("position")), "size": deepcopy(bin_obj.get("outer")),
                    "positionStatus": bin_obj.get("positionStatus"),
                })
                half_x, half_y = self._entity_footprint(bin_obj)
                occupancy.append({
                    "kind": "bin", "id": bin_obj["id"], "center": deepcopy(bin_obj.get("position")),
                    "halfExtents": {"x": half_x, "y": half_y},
                })
            relationships = []
            availability_warnings = []
            for entity in entities:
                position = entity.get("position") or {}
                if workspace.get("available"):
                    containing = []
                    for name, region in (workspace.get("regions") or {}).items():
                        if (
                            float(region["xMin"]) <= float(position.get("x", 0.0)) <= float(region["xMax"])
                            and float(region["yMin"]) <= float(position.get("y", 0.0)) <= float(region["yMax"])
                        ):
                            containing.append(name)
                    relationships.append({"entityId": entity["id"], "regions": containing})
                    if not containing:
                        availability_warnings.append({
                            "entityId": entity["id"],
                            "code": "outside_calibrated_workspace",
                            "message": f"{entity.get('label') or entity['id']} is outside the calibrated workspace bounds.",
                        })
                if entity.get("kind") == "bin" and entity.get("positionStatus") == "simulation_only":
                    availability_warnings.append({
                        "entityId": entity["id"],
                        "code": "simulation_only",
                        "message": f"{entity.get('label') or entity['id']} has only a simulated position and is not physically verified.",
                    })
            registered_inventory = []
            for definition in self.registered_parts.values():
                visible = definition["partId"] in self.parts
                registered_inventory.append({
                    "partId": definition["partId"], "tagId": definition.get("tagId"),
                    "label": definition.get("label"), "visible": visible,
                })
                if not visible:
                    availability_warnings.append({
                        "entityId": definition["partId"],
                        "code": "tag_not_visible",
                        "message": f"{definition.get('label') or definition['partId']} is registered but its AprilTag is not currently visible, so it has no usable position.",
                    })
            workspace_markers = [
                {
                    "id": marker.get("id"), "center": deepcopy(marker.get("center")),
                    "sizeM": float(marker.get("sizeM") or 0.05),
                }
                for marker in (((self.calibration or {}).get("fiducials") or {}).get("referenceMarkers") or [])
            ]
            return {
                "ok": True, "frame": "robot_base_meters", "workspace": workspace,
                "entities": entities, "taughtPoints": deepcopy(list(self.taught_points.values())),
                "registeredInventory": registered_inventory,
                "relationships": relationships,
                "availabilityWarnings": availability_warnings,
                "occupancy": occupancy,
                "workspaceMarkers": workspace_markers,
                "robotBaseExclusion": {
                    "center": {"x": 0.0, "y": 0.0},
                    "radiusM": BASE_TRANSFER_CLEARANCE_RADIUS_M,
                },
            }

    # ------------------------------------------------------------ snapshot

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return self.snapshot_locked()

    def snapshot_locked(self) -> Dict[str, Any]:
        parts = [self._part_with_camera_status(part) for part in self.parts.values()]
        registered = []
        for definition in self.registered_parts.values():
            item = deepcopy(definition)
            visible = item["partId"] in self.parts and self.parts[item["partId"]].get("trackingMode") == "apriltag"
            item["visible"] = visible
            if visible:
                item["position"] = deepcopy(self.parts[item["partId"]]["position"])
                item["orientationDeg"] = self.parts[item["partId"]].get("orientationDeg", 0.0)
            else:
                item.pop("position", None)
                item.pop("orientationDeg", None)
            registered.append(item)
        return {
            "ok": True,
            "frame": "robot_base_meters",
            "version": self.version,
            "updatedAt": self.updated_at,
            "parts": parts,
            "registeredParts": registered,
            "tagTrackRevision": self.tag_track_revision,
            "bins": [self._bin_with_geometry(b) for b in self.bins.values()],
            "taughtPoints": deepcopy(list(self.taught_points.values())),
            "workspaceRegions": self.workspace_regions_locked(),
            "programs": list(self.programs.values()),
            "calibration": self.calibration,
            "camera": self.camera,
            "coordinatePlanner": self.coordinate_planner,
            "endEffector": self.end_effector,
            "endEffectors": [
                {"id": key, "label": label}
                for key, label in END_EFFECTORS.items()
            ],
            # Back-compat for the Realtime agent tools.
            "objects": parts,
        }

    def _part_with_camera_status(self, part: Dict[str, Any]) -> Dict[str, Any]:
        item = dict(part)
        if item.get("source") == "camera":
            detected_at = float(item.get("detectedAt") or item.get("updatedAt") or 0.0)
            stale_after = float(self.camera.get("staleAfterS") or 3.0)
            item["stale"] = bool(detected_at and time.time() - detected_at > stale_after)
            item["ageS"] = round(max(0.0, time.time() - detected_at), 2) if detected_at else None
        return item

    def clear_parts(self) -> Dict[str, Any]:
        with self.lock:
            self.parts = {}
            self._save_locked()
            return self.snapshot_locked()

    def set_end_effector(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            self.end_effector = self._normalize_end_effector(body.get("endEffector") or body.get("id"))
            self._save_locked()
            return self.snapshot_locked()

    def set_coordinate_planner_config(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            config = dict(self.coordinate_planner or self._default_coordinate_planner())
            if "toolRpyDeg" in body:
                rpy = body.get("toolRpyDeg")
                if rpy is None:
                    config["toolRpyDeg"] = None
                    config["toolRpySource"] = "missing"
                elif isinstance(rpy, dict):
                    config["toolRpyDeg"] = {
                        "rx": float(rpy.get("rx", rpy.get("roll", 0.0))),
                        "ry": float(rpy.get("ry", rpy.get("pitch", 0.0))),
                        "rz": float(rpy.get("rz", rpy.get("yaw", 0.0))),
                    }
                    config["toolRpySource"] = str(body.get("toolRpySource") or "captured")
                elif isinstance(rpy, list) and len(rpy) >= 3:
                    config["toolRpyDeg"] = {
                        "rx": float(rpy[0]),
                        "ry": float(rpy[1]),
                        "rz": float(rpy[2]),
                    }
                    config["toolRpySource"] = str(body.get("toolRpySource") or "captured")
                else:
                    return {"ok": False, "error": "toolRpyDeg must be null, {rx,ry,rz}, or a 3-value list."}
            config["updatedAt"] = time.time()
            if "pickHeightBiasM" in body:
                config["pickHeightBiasM"] = clamp(body.get("pickHeightBiasM"), -0.008, 0.008)
            if "minimumTableClearanceM" in body:
                config["minimumTableClearanceM"] = clamp(
                    body.get("minimumTableClearanceM"), 0.002, 0.012
                )
            profiles = deepcopy(config.get("toolProfiles") or DEFAULT_TOOL_PROFILES)
            if "toolTcpCorrectionLocalM" in body:
                raw_correction = body.get("toolTcpCorrectionLocalM") or {}
                profile = profiles.setdefault(self.end_effector, deepcopy(DEFAULT_TOOL_PROFILES[self.end_effector]))
                profile["tcpCorrectionLocalM"] = {
                    axis: clamp(raw_correction.get(axis, 0.0), -0.03, 0.03)
                    for axis in ("x", "y", "z")
                }
            if "observedContactMissMm" in body:
                miss = body.get("observedContactMissMm") or {}
                # Robot axes: +X forward, +Y left, +Z high. Convert the
                # observed physical TCP error into the canonical top-down tool
                # frame; adding it to the flange->TCP model makes the outgoing
                # flange target compensate in the opposite direction.
                yaw_deg = float(body.get("calibrationJawYawDeg") or 0.0)
                tcp_rotation = top_down_tcp_rotation(yaw_deg)
                error_world = (
                    clamp(miss.get("forward", 0.0), -30.0, 30.0) / 1000.0,
                    clamp(miss.get("left", 0.0), -30.0, 30.0) / 1000.0,
                    clamp(miss.get("high", 0.0), -30.0, 30.0) / 1000.0,
                )
                correction_local = tuple(
                    sum(tcp_rotation[row][column] * error_world[row] for row in range(3))
                    for column in range(3)
                )
                profile = profiles.setdefault(self.end_effector, deepcopy(DEFAULT_TOOL_PROFILES[self.end_effector]))
                previous = profile.get("tcpCorrectionLocalM") or {}
                profile["tcpCorrectionLocalM"] = {
                    axis: clamp(float(previous.get(axis, 0.0)) + correction_local[index], -0.03, 0.03)
                    for index, axis in enumerate(("x", "y", "z"))
                }
            if "suctionInstalledGeometry" in body:
                raw_geometry = body.get("suctionInstalledGeometry") or {}
                suction = profiles.setdefault("suction_gripper", deepcopy(DEFAULT_TOOL_PROFILES["suction_gripper"]))
                geometry = {**DEFAULT_TOOL_PROFILES["suction_gripper"]["geometry"], **(suction.get("geometry") or {})}
                if "flangeToCupStartM" in raw_geometry:
                    geometry["flangeToCupStartM"] = clamp(raw_geometry["flangeToCupStartM"], 0.02, 0.10)
                if "cupFreeExtensionM" in raw_geometry:
                    geometry["cupFreeExtensionM"] = clamp(raw_geometry["cupFreeExtensionM"], 0.005, 0.05)
                if "cupDiameterM" in raw_geometry:
                    geometry["cupDiameterM"] = clamp(raw_geometry["cupDiameterM"], 0.01, 0.05)
                geometry["flangeToContactM"] = round(
                    float(geometry["flangeToCupStartM"]) + float(geometry["cupFreeExtensionM"]), 6
                )
                suction["geometry"] = geometry
            config["toolProfiles"] = profiles
            config.pop("toolOffsetMode", None)
            config.pop("legacyToolOffsetMode", None)
            config.pop("toolVerticalLiftM", None)
            config["toolOffsetsM"] = deepcopy(TOOL_TCP_OFFSETS_M)
            self.coordinate_planner = config
            self._save_locked()
            return {"ok": True, "coordinatePlanner": self.coordinate_planner, **self.snapshot_locked()}

    def match_part(self, query: str) -> Optional[Dict[str, Any]]:
        with self.lock:
            parts = [deepcopy(part) for part in self.parts.values()]
            if not parts:
                return None
            tokens = {t for t in str(query).lower().replace("-", " ").split() if t}

            def score(part: Dict[str, Any]) -> int:
                label = str(part.get("label", "")).lower()
                value = 0
                if part["id"].lower() in tokens:
                    value += 6
                if label and any(t in label for t in tokens):
                    value += 4
                if str(part.get("type", "")).lower() in tokens:
                    value += 2
                return value

            ranked = sorted(parts, key=score, reverse=True)
            return ranked[0] if score(ranked[0]) > 0 else parts[0]

    # ------------------------------------------------------------ programs

    def normalized_program(self, body: Dict[str, Any]) -> Dict[str, Any]:
        program_id = str(body.get("id") or self._next_id("prog"))
        steps = []
        for index, raw in enumerate(body.get("steps") or []):
            if not isinstance(raw, dict):
                continue
            step_type = str(raw.get("type") or "").lower()
            step_id = str(raw.get("id") or f"{program_id}-step-{index + 1}")
            common = {
                "id": step_id,
                "label": str(raw.get("label") or "").strip() or None,
                "enabled": bool(raw.get("enabled", True)),
            }
            if step_type == "pick" and raw.get("objectId"):
                steps.append({**common, "type": "pick", "objectId": str(raw["objectId"])})
            elif step_type == "place" and raw.get("binId"):
                steps.append({**common, "type": "place", "binId": str(raw["binId"])})
            elif step_type == "place" and raw.get("position"):
                p = raw["position"]
                steps.append({
                    **common,
                    "type": "place",
                    "position": {
                        "x": clamp(p.get("x", 0.2), -SCENE_BOUND_M, SCENE_BOUND_M),
                        "y": clamp(p.get("y", 0.0), -SCENE_BOUND_M, SCENE_BOUND_M),
                        "z": clamp(p.get("z", 0.0), 0.0, 0.3),
                    },
                })
            elif step_type == "place" and raw.get("pointId"):
                steps.append({**common, "type": "place", "pointId": str(raw["pointId"])})
            elif step_type in ("move", "move_to_point") and (raw.get("pointId") or raw.get("waypoint")):
                motion_type = str(raw.get("motionType") or ("joint" if step_type == "move_to_point" else "joint")).lower()
                if motion_type not in ("joint", "linear"):
                    motion_type = "joint"
                move = {
                    **common,
                    "type": "move",
                    "motionType": motion_type,
                    "speed": int(clamp(raw.get("speed", 20), 1, 100)),
                }
                if raw.get("pointId"):
                    move["pointId"] = str(raw["pointId"])
                if isinstance(raw.get("waypoint"), dict):
                    move["waypoint"] = deepcopy(raw["waypoint"])
                steps.append(move)
            elif step_type == "tool":
                action = str(raw.get("action") or "").lower()
                if action in ("acquire", "release"):
                    steps.append({**common, "type": "tool", "action": action})
            elif step_type in ("acquire", "grip", "suction_on"):
                steps.append({**common, "type": "tool", "action": "acquire"})
            elif step_type in ("release", "open", "suction_off"):
                steps.append({**common, "type": "tool", "action": "release"})
            elif step_type == "wait":
                steps.append({
                    **common,
                    "type": "wait",
                    "durationMs": int(clamp(raw.get("durationMs", 1000), 50, 600000)),
                })
            elif step_type == "home":
                steps.append({**common, "type": "home"})
        return {
            "id": program_id,
            "name": str(body.get("name") or program_id),
            "editorVersion": 2,
            "repeatCount": int(clamp(body.get("repeatCount", 1), 1, 20)),
            "steps": steps,
            "updatedAt": time.time(),
        }

    def save_program(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            program = self.normalized_program(body)
            error = self._validate_program_locked(program)
            if error:
                return {"ok": False, "error": error}
            # Older API and Realtime callers still submit the v1 sequential
            # schema. Keep their persisted representation untouched until an
            # operator opens and saves it in the v2 programmer. Loading always
            # migrates a legacy record in memory through normalized_program.
            if int(body.get("editorVersion") or 1) < 2:
                legacy_steps: List[Dict[str, Any]] = []
                for raw in body.get("steps") or []:
                    if not isinstance(raw, dict):
                        continue
                    kind = str(raw.get("type") or "")
                    if kind in ("pick", "place", "home", "move_to_point", "acquire", "release"):
                        legacy_steps.append(deepcopy(raw))
                stored_program = {
                    "id": program["id"],
                    "name": program["name"],
                    "steps": legacy_steps,
                    "updatedAt": program["updatedAt"],
                }
            else:
                stored_program = program
            self.programs[program["id"]] = stored_program
            self._save_locked()
            return {**self.snapshot_locked(), "program": stored_program}

    def delete_program(self, program_id: str) -> Dict[str, Any]:
        with self.lock:
            normalized_id = str(program_id)
            if not normalized_id or normalized_id not in self.programs:
                return {"ok": False, "error": f"Program '{normalized_id or 'unknown'}' was not found."}
            self.programs.pop(normalized_id)
            self._save_locked()
            return {**self.snapshot_locked(), "ok": True, "deletedProgramId": normalized_id}

    def _validate_program_locked(self, program: Dict[str, Any]) -> Optional[str]:
        holding: Optional[str] = None
        enabled_steps = [step for step in program["steps"] if step.get("enabled", True)]
        if not enabled_steps:
            return "Program has no steps."
        for index, step in enumerate(enabled_steps, start=1):
            if step["type"] == "pick":
                if holding:
                    return f"Step {index}: still holding {holding}; place it before picking again."
                if step["objectId"] not in self.parts:
                    return f"Step {index}: part '{step['objectId']}' is not in the scene."
                holding = step["objectId"]
            elif step["type"] == "place":
                if not holding:
                    return f"Step {index}: place without a held part; add a pick first."
                if step.get("binId") and step["binId"] not in self.bins:
                    return f"Step {index}: bin '{step['binId']}' is not in the scene."
                if step.get("pointId") and step["pointId"] not in self.taught_points:
                    return f"Step {index}: taught point '{step['pointId']}' is not available."
                holding = None
            elif step["type"] == "move":
                point = self.taught_points.get(str(step.get("pointId") or "")) if step.get("pointId") else step.get("waypoint")
                if not isinstance(point, dict):
                    return f"Step {index}: motion has no captured or shared waypoint."
                if step.get("pointId") and "waypoint" not in point.get("uses", []):
                    return f"Step {index}: taught point '{point.get('label')}' is not enabled as a waypoint."
                if len(point.get("jointAnglesDeg") or []) != 6 or len(point.get("firmwareFlangeCoordsMmDeg") or []) != 6:
                    return f"Step {index}: waypoint '{point.get('label') or step['id']}' has incomplete robot data."
            elif step["type"] in ("tool", "wait", "home"):
                continue
        if holding:
            return f"Program ends still holding {holding}; add a place step."
        return None

    # ------------------------------------------------------------- camera

    def set_camera_config(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            camera = dict(self.camera or self._default_camera())
            for key in (
                "deviceId",
                "deviceUniqueId",
                "deviceLabel",
                "enabled",
                "width",
                "height",
                "jpegQuality",
                "staleAfterS",
                "localization",
            ):
                if key in body:
                    camera[key] = body[key]
            bounds = body.get("workspaceBounds")
            if isinstance(bounds, dict):
                current = dict(camera.get("workspaceBounds") or {})
                for key in ("xMin", "xMax", "yMin", "yMax", "zMin", "zMax"):
                    if key in bounds:
                        current[key] = float(bounds[key])
                camera["workspaceBounds"] = current
            camera["updatedAt"] = time.time()
            localization = camera.get("localization") or {}
            activation_error = None
            if localization.get("enabled"):
                calibration = self.calibration or {}
                intrinsics = calibration.get("intrinsics") or {}
                fiducials = calibration.get("fiducials") or {}
                verification = calibration.get("verification") or {}
                if not intrinsics.get("ok") or float(intrinsics.get("intrinsicRmsPx") or float("inf")) > 2.5 or float(intrinsics.get("maximumViewErrorPx") or intrinsics.get("intrinsicRmsPx") or float("inf")) > 4.0:
                    activation_error = "Continuous localization requires a passing intrinsic calibration."
                elif not fiducials.get("baselineHomography"):
                    activation_error = "Continuous localization requires a locked camera pose."
                elif not (verification.get("passed") or verification.get("testingBypass")):
                    activation_error = "Continuous localization requires accuracy verification or explicit testing mode."
                if activation_error:
                    camera["localization"] = {**localization, "enabled": False}
            self.camera = camera
            self._save_locked()
            return {**self.snapshot_locked(), "ok": activation_error is None, "error": activation_error, "camera": self.camera}

    def physical_program_gate_error(self) -> Optional[str]:
        """Return hard camera-calibration failures that prohibit execution.

        Choosing the explicit testing bypass no longer counts as a hard
        failure. Fresh tag observations, stale-preview detection, the physical
        confirmation token, and motion feedback remain mandatory.
        """
        return None

    def physical_program_warning(self) -> Optional[str]:
        """Describe the reduced accuracy assurance of testing bypass mode."""
        with self.lock:
            verification = (self.calibration or {}).get("verification") or {}
            if verification.get("testingBypass") and not verification.get("passed"):
                return (
                    "Camera coordinates have not passed the optional nine-point accuracy check. "
                    "Running with the accepted fiducial calibration in testing mode."
                )
        return None

    def validate_plan_object_snapshots(self, plan: Dict[str, Any], tolerance_m: float = 0.005) -> Optional[str]:
        """Reject a preview when its camera target moved after planning."""
        with self.lock:
            for snapshot in plan.get("objectSnapshots") or []:
                object_id = str(snapshot.get("objectId") or "")
                current = self.parts.get(object_id)
                if current is None:
                    return f"Planned object '{object_id}' is no longer visible; plan again."
                if current.get("trackingMode") == "apriltag" and time.time() - float(current.get("lastSeenAt") or 0.0) > 1.0:
                    return f"{current.get('label') or object_id} does not have a fresh AprilTag pose; plan again."
                planned = snapshot.get("position") or {}
                position = current.get("position") or {}
                distance = math.hypot(float(position.get("x", 0.0)) - float(planned.get("x", 0.0)),
                                      float(position.get("y", 0.0)) - float(planned.get("y", 0.0)))
                if distance > tolerance_m:
                    return f"{current.get('label') or object_id} moved {distance * 1000:.1f} mm after planning; plan again."
            for snapshot in plan.get("destinationSnapshots") or []:
                destination_id = str(snapshot.get("id") or "")
                if snapshot.get("kind") == "bin":
                    current_bin = self.bins.get(destination_id)
                    if current_bin is None:
                        return f"Destination bin '{destination_id}' no longer exists; plan again."
                    if current_bin.get("positionStatus") != "operator_verified":
                        return (
                            f"{current_bin.get('label') or destination_id} exists only at a simulated position. "
                            "Move the real bin there and confirm its position before running."
                        )
                    planned_position = snapshot.get("position") or {}
                    current_position = current_bin.get("position") or {}
                    if math.hypot(
                        float(current_position.get("x", 0.0)) - float(planned_position.get("x", 0.0)),
                        float(current_position.get("y", 0.0)) - float(planned_position.get("y", 0.0)),
                    ) > tolerance_m:
                        return f"{current_bin.get('label') or destination_id} moved after planning; plan again."
                elif snapshot.get("kind") == "point":
                    current_point = self.taught_points.get(destination_id)
                    if current_point is None:
                        return f"Linked taught point '{destination_id}' was deleted; plan again."
                    if float(current_point.get("updatedAt") or 0.0) != float(snapshot.get("updatedAt") or 0.0):
                        return (
                            f"Linked taught point '{current_point.get('label') or destination_id}' "
                            "changed after planning; plan again."
                        )
                    if current_point.get("toolCalibrationFingerprint") != self.tool_calibration_fingerprint(
                        current_point.get("endEffector")
                    ):
                        return (
                            f"Linked taught point '{current_point.get('label') or destination_id}' "
                            "uses an outdated tool calibration; recapture it."
                        )
            for step in plan.get("steps") or []:
                if step.get("waypointSource") != "embedded":
                    continue
                waypoint_id = str(step.get("waypointId") or step.get("pointId") or "embedded waypoint")
                step_id = str(step.get("sourceStepId") or step.get("stateId") or "unknown step")
                label = str(step.get("pointLabel") or waypoint_id)
                joint_values = step.get("jointTargetDeg")
                flange_values = step.get("capturedFlangeCoordsMmDeg")
                coordinate_values = step.get("coordsMm")
                preferred_values = step.get("preferredJointSeedDeg")
                complete = (
                    isinstance(joint_values, list)
                    and len(joint_values) == 6
                    and isinstance(flange_values, list)
                    and len(flange_values) == 6
                ) or (
                    isinstance(coordinate_values, list)
                    and len(coordinate_values) == 6
                    and isinstance(preferred_values, list)
                    and len(preferred_values) == 6
                )
                if not complete:
                    return (
                        f"Embedded waypoint '{label}' in program step '{step_id}' has incomplete "
                        "joint or flange data; teach the point again."
                    )
                active_tool = str(step.get("activeTool") or self.end_effector)
                if active_tool != self.end_effector:
                    return (
                        f"Embedded waypoint '{label}' in program step '{step_id}' was captured for "
                        f"{active_tool}; select that tool or teach the point again."
                    )
                fingerprint = str(step.get("capturedToolCalibrationFingerprint") or "")
                if not fingerprint or fingerprint != self.tool_calibration_fingerprint(active_tool):
                    return (
                        f"Embedded waypoint '{label}' in program step '{step_id}' uses an outdated "
                        "tool calibration; teach the point again."
                    )
        return None

    def release_plan_reservations(self, plan: Dict[str, Any]) -> None:
        with self.lock:
            changed = False
            for snapshot in plan.get("objectSnapshots") or []:
                part = self.parts.get(str(snapshot.get("objectId") or ""))
                if part and part.get("reservedByPlan"):
                    part.pop("reservedByPlan", None)
                    part.pop("reservationCreatedAt", None)
                    changed = True
            if changed:
                self._save_locked()

    def set_calibration(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            current = dict(self.calibration or self._default_calibration())
            previous_intrinsics = current.get("intrinsics")
            previous_fiducials = current.get("fiducials") or {}
            current_pose = current.get("cameraToRobot") or {}
            position = body.get("position") or current_pose.get("position") or {}
            rpy = body.get("rpyDeg") or current_pose.get("rpyDeg") or {}
            homography = body.get("homography") if isinstance(body.get("homography"), list) else None
            points = body.get("calibrationPoints") if isinstance(body.get("calibrationPoints"), dict) else None
            next_fiducials = {
                **deepcopy(current.get("fiducials") or self._default_calibration()["fiducials"]),
                **deepcopy(body.get("fiducials") or {}),
            }
            geometry_keys = ("dictionary", "markerSizeM", "referenceMarkers")
            previous_geometry = {key: previous_fiducials.get(key) for key in geometry_keys}
            next_geometry = {key: next_fiducials.get(key) for key in geometry_keys}
            intrinsics_changed = "intrinsics" in body and body.get("intrinsics") != previous_intrinsics
            geometry_changed = "fiducials" in body and next_geometry != previous_geometry
            if intrinsics_changed or geometry_changed:
                next_fiducials["baselineHomography"] = None
                next_fiducials["allowCurrentPose"] = False
            self.calibration = {
                **current,
                "status": "configured",
                "mode": str(body.get("mode") or ("table_homography" if homography else current.get("mode") or "extrinsic_pose")),
                "cameraId": body.get("cameraId", self.camera.get("deviceId")),
                "cameraToRobot": {
                    "position": {
                        "x": float(position.get("x", 0.0)),
                        "y": float(position.get("y", 0.0)),
                        "z": float(position.get("z", 0.4)),
                    },
                    "rpyDeg": {
                        "roll": float(rpy.get("roll", 0.0)),
                        "pitch": float(rpy.get("pitch", 0.0)),
                        "yaw": float(rpy.get("yaw", 0.0)),
                    },
                },
                "homography": homography if homography is not None else current.get("homography"),
                "calibrationPoints": points if points is not None else current.get("calibrationPoints"),
                # API handlers enrich their response after this call. Keep a
                # detached copy so that enrichment cannot create a cycle in
                # the persisted in-memory calibration graph.
                "intrinsics": deepcopy(body.get("intrinsics", current.get("intrinsics"))),
                "fiducials": next_fiducials,
                "verification": None if (intrinsics_changed or geometry_changed) else body.get("verification", current.get("verification")),
                "note": "Detections in frame='camera' are transformed with this pose.",
                "updatedAt": time.time(),
            }
            self._save_locked()
            return {"ok": True, "calibration": self.calibration}

    def verify_executed_steps(self, result: Dict[str, Any]) -> Dict[str, Any]:
        """Annotate physical execution using fresh camera detections when possible."""
        executed = result.get("executedSteps") or []
        if not executed:
            return result
        stale_after = float((self.camera or {}).get("staleAfterS") or 3.0)
        now = time.time()
        with self.lock:
            parts = {pid: dict(part) for pid, part in self.parts.items()}

        for step in executed:
            object_id = step.get("objectId") or step.get("attachObjectId") or step.get("releaseObjectId")
            if not object_id:
                continue
            part = parts.get(str(object_id))
            if not part or part.get("source") != "camera":
                continue
            detected_at = float(part.get("detectedAt") or part.get("updatedAt") or 0.0)
            fresh = bool(detected_at and now - detected_at <= stale_after)
            verification = {
                "expectedObjectId": object_id,
                "verified": None,
                "verificationReason": "no_fresh_camera_detection",
                "beforeDetection": part,
                "afterDetection": part if fresh else None,
            }
            if step.get("name") == "auto_grip" and fresh:
                verification["verified"] = False
                verification["verificationReason"] = "object_still_visible_at_pick_after_grip"
                step["preventSceneMove"] = True
            elif step.get("releaseObjectId") and fresh:
                placed = step.get("placedPosition") or {}
                dx = float(part["position"]["x"]) - float(placed.get("x", part["position"]["x"]))
                dy = float(part["position"]["y"]) - float(placed.get("y", part["position"]["y"]))
                dist = math.hypot(dx, dy)
                verification["verified"] = dist <= 0.06
                verification["verificationReason"] = "object_seen_near_place" if dist <= 0.06 else "object_seen_away_from_place"
                verification["xyErrorM"] = round(dist, 4)
                if dist > 0.06:
                    step["preventSceneMove"] = True
            step["verification"] = verification
        return result

    # ------------------------------------------------------------- planner

    def plan_program(
        self,
        steps: List[Dict[str, Any]],
        start_angles: List[float],
        program_name: str = "ad-hoc",
        repeat_count: int = 1,
    ) -> Dict[str, Any]:
        """Expand scene and taught-point instructions into a coordinate plan."""
        with self.lock:
            # Coordinate-mode execution uses the robot firmware coordinate
            # frame directly. Do not apply the old scene-to-planning Z offset;
            # object and bin Z values are already robot-frame table heights.
            parts = {pid: deepcopy(p) for pid, p in self.parts.items()}
            registered_parts = {pid: deepcopy(p) for pid, p in self.registered_parts.items()}
            bins = {bid: deepcopy(b) for bid, b in self.bins.items()}
            points = {pid: deepcopy(p) for pid, p in self.taught_points.items()}

        plan_steps: List[Dict[str, Any]] = []
        notes: List[str] = []
        object_snapshots: Dict[str, Dict[str, Any]] = {}
        destination_snapshots: Dict[str, Dict[str, Any]] = {}
        sequence = 0
        rpy = self._coordinate_tool_rpy()
        rpy_source = "canonical_top_down" if self.end_effector == "adaptive_gripper" else ("captured" if rpy is not None else "missing")

        index = 0
        previous_taught_point: Optional[Dict[str, Any]] = None
        base_instructions = [deepcopy(step) for step in steps if step.get("enabled", True)]
        repeat_count = int(clamp(repeat_count, 1, 20))
        instructions: List[Dict[str, Any]] = []
        for iteration in range(repeat_count):
            for source_index, source in enumerate(base_instructions):
                instruction = deepcopy(source)
                instruction["_sourceIndex"] = source_index
                instruction["_iteration"] = iteration + 1
                instructions.append(instruction)
        while index < len(instructions):
            instruction = instructions[index]
            kind = instruction.get("type")
            if kind == "pick":
                requested_object_id = str(instruction.get("objectId"))
                part = parts.get(requested_object_id)
                if part is None:
                    registered = registered_parts.get(requested_object_id)
                    if registered is not None:
                        return {
                            "ok": False,
                            "error": (
                                f"{registered.get('label') or requested_object_id} is registered, but its "
                                "AprilTag is not currently visible."
                            ),
                        }
                    return {"ok": False, "error": f"Pick target '{instruction.get('objectId')}' not found."}
                if part.get("trackingMode") == "apriltag" and time.time() - float(part.get("lastSeenAt") or 0.0) > 1.0:
                    return {"ok": False, "error": f"{part['label']} is registered but its AprilTag is not currently visible."}
                if part["id"] not in object_snapshots:
                    object_snapshots[part["id"]] = {
                        "objectId": part["id"], "label": part["label"],
                        "position": deepcopy(part["position"]), "size": deepcopy(part["size"]),
                        "detectedAt": part.get("detectedAt"), "trackId": part.get("trackId"),
                        "lastSeenAt": part.get("lastSeenAt"), "trackingMode": part.get("trackingMode"),
                    }
                place = instructions[index + 1] if index + 1 < len(instructions) else None
                if not place or place.get("type") != "place":
                    return {"ok": False, "error": f"Pick of {part['label']} must be followed by a place step."}
                destination = self._resolve_destination(place, bins, part, points)
                if destination is None:
                    return {"ok": False, "error": f"Place destination for {part['label']} not found."}
                if destination.get("kind") == "bin":
                    destination_bin = destination["bin"]
                    destination_snapshots[destination_bin["id"]] = {
                        "kind": "bin",
                        "id": destination_bin["id"],
                        "label": destination_bin.get("label"),
                        "position": deepcopy(destination_bin.get("position")),
                        "positionStatus": destination_bin.get("positionStatus", "operator_verified"),
                    }
                    if destination_bin.get("positionStatus") == "simulation_only":
                        notes.append(
                            f"{destination_bin['label']} was moved only in the simulation; "
                            "confirm its physical position before running this plan."
                        )
                elif place.get("pointId"):
                    point_snapshot = points.get(str(place.get("pointId")))
                    if point_snapshot:
                        destination_snapshots[point_snapshot["id"]] = {
                            "kind": "point", "id": point_snapshot["id"],
                            "label": point_snapshot.get("label"),
                            "updatedAt": point_snapshot.get("updatedAt"),
                            "toolCalibrationFingerprint": point_snapshot.get("toolCalibrationFingerprint"),
                        }
                sequence += 1
                segment = self._plan_single_pick_coordinate(
                    part, destination, sequence, rpy, rpy_source,
                    route_low_approach=True,
                )
                if not segment["ok"]:
                    return {
                        "ok": False,
                        "error": f"{part['label']}: {segment['error']}",
                        "failedInstruction": index + 1,
                        "steps": plan_steps,
                    }
                for step in segment["steps"]:
                    step["sourceStepId"] = instruction.get("id")
                    step["sourceStepIds"] = [
                        instruction.get("id"),
                        place.get("id"),
                    ]
                    step["sourceIteration"] = instruction.get("_iteration", 1)
                    if step.get("placedPosition"):
                        step["placedPosition"] = dict(segment["placedPosition"])
                plan_steps.extend(segment["steps"])
                notes.extend(segment.get("notes", []))
                # Virtual scene update so the next pick plans around the move.
                part["position"] = dict(segment["placedPosition"])
                previous_taught_point = None
                index += 2
                continue
            if kind in ("move", "move_to_point"):
                linked_point_id = str(instruction.get("pointId") or "")
                point = (
                    points.get(linked_point_id)
                    if linked_point_id
                    else deepcopy(instruction.get("waypoint"))
                )
                if point is None:
                    return {"ok": False, "error": f"Taught point '{instruction.get('pointId')}' not found."}
                sequence += 1
                segment = self._taught_point_motion(
                    point,
                    sequence,
                    previous_taught_point,
                    motion_type=(
                        str(instruction.get("motionType") or "joint")
                        if kind == "move"
                        else "legacy"
                    ),
                    speed=int(clamp(instruction.get("speed", 20), 1, 100)),
                )
                if not segment.get("ok"):
                    return {"ok": False, "error": segment.get("error"), "failedInstruction": index + 1, "steps": plan_steps}
                point_key = str(point.get("id") or instruction.get("id") or f"embedded-{sequence}")
                waypoint_source = "linked" if linked_point_id else "embedded"
                for step in segment.get("steps") or []:
                    step["sourceStepId"] = instruction.get("id")
                    step["sourceIteration"] = instruction.get("_iteration", 1)
                    step["waypointSource"] = waypoint_source
                    step["waypointId"] = point_key
                plan_steps.extend(segment.get("steps") or [])
                notes.extend(segment.get("notes") or [])
                if linked_point_id:
                    destination_snapshots[point_key] = {
                        "kind": "point", "id": point_key, "label": point.get("label"),
                        "updatedAt": point.get("updatedAt"),
                        "toolCalibrationFingerprint": point.get("toolCalibrationFingerprint"),
                    }
                previous_taught_point = point
                index += 1
                continue
            if kind in ("tool", "acquire", "release"):
                sequence += 1
                action = str(
                    instruction.get("action")
                    or ("release" if kind == "release" else "acquire")
                )
                acquiring = action == "acquire"
                plan_steps.append({
                    "stateId": f"seq{sequence:02d}_{action}",
                    "name": "auto_grip" if acquiring else "release_gripper",
                    "gripper": "closed" if acquiring else "open",
                    "gripperAction": "auto_grip" if acquiring else "open_at_drop",
                    "gripperActionTiming": "after_arrival",
                    "activeTool": self.end_effector,
                    "durationMs": 1100 if acquiring else 1000,
                    "sourceStepId": instruction.get("id"),
                    "sourceIteration": instruction.get("_iteration", 1),
                })
                index += 1
                continue
            if kind == "wait":
                sequence += 1
                duration_ms = int(clamp(instruction.get("durationMs", 1000), 50, 600000))
                plan_steps.append({
                    "stateId": f"seq{sequence:02d}_wait",
                    "name": "wait",
                    "waitMs": duration_ms,
                    "durationMs": duration_ms,
                    "sourceStepId": instruction.get("id"),
                    "sourceIteration": instruction.get("_iteration", 1),
                })
                index += 1
                continue
            if kind == "home":
                sequence += 1
                plan_steps.append({
                    "stateId": f"seq{sequence:02d}_home",
                    "name": "home",
                    "robotCommand": "home",
                    "durationMs": 4500,
                    "sourceStepId": instruction.get("id"),
                    "sourceIteration": instruction.get("_iteration", 1),
                })
                previous_taught_point = None
                index += 1
                continue
            if kind == "place":
                return {"ok": False, "error": "Place step without a preceding pick."}
            index += 1

        if not plan_steps:
            return {"ok": False, "error": "Program produced no motion."}

        total_ms = sum(int(s.get("durationMs") or 0) for s in plan_steps)
        requires_captured_tool_rpy = any(
            isinstance(step.get("coordsMm"), list) and any(value is None for value in step["coordsMm"][3:6])
            for step in plan_steps
        )
        preflight_errors: List[Dict[str, Any]] = []
        for step in plan_steps:
            if step.get("coordsMm") is not None:
                preflight_errors.extend(validate_coordinate_bounds(
                    step.get("coordsMm"),
                    str(step.get("stateId") or step.get("name") or "unknown"),
                    allow_missing_rpy=True,
                ))
        if preflight_errors:
            first = preflight_errors[0]
            message = first.get("message") or first.get("error") or "coordinate preflight failed"
            notes.append(f"Coordinate preflight failed: {message}")
        unverified_destinations = [
            snapshot for snapshot in destination_snapshots.values()
            if snapshot.get("kind") == "bin" and snapshot.get("positionStatus") != "operator_verified"
        ]
        physical_ready = not requires_captured_tool_rpy and not preflight_errors and not unverified_destinations
        with self.lock:
            for existing in self.parts.values():
                existing.pop("reservedByPlan", None)
                existing.pop("reservationCreatedAt", None)
            for object_id in object_snapshots:
                if object_id in self.parts:
                    self.parts[object_id]["reservedByPlan"] = program_name
                    self.parts[object_id]["reservationCreatedAt"] = time.time()
            if object_snapshots:
                self._save_locked()
        return {
            "ok": True,
            "mode": "coordinate_program",
            "program": program_name,
            "repeatCount": repeat_count,
            "steps": plan_steps,
            "reachable": True,
            "physicalReady": physical_ready,
            "requiresCapturedToolRpy": requires_captured_tool_rpy,
            "coordinatePreflight": {
                "ok": not preflight_errors,
                "limits": deepcopy(MYCOBOT_280_COORD_LIMITS),
                "errors": preflight_errors,
            },
            "durationMs": total_ms,
            "sceneRevision": self.version,
            "objectSnapshots": list(object_snapshots.values()),
            "destinationSnapshots": list(destination_snapshots.values()),
            "unverifiedDestinations": unverified_destinations,
            "motionModel": {
                "type": "firmware_coordinate",
                "planner": "robot_send_coords",
                "toolRpySource": rpy_source,
                "activeEndEffector": self.end_effector,
                "configuredToolTcpOffsetM": deepcopy(self._configured_tool_tcp_offset()),
                "pickHeightBiasM": round(self._pick_height_bias_m(), 4),
                "toolRpyDeg": (
                    {"rx": round(rpy[0], 3), "ry": round(rpy[1], 3), "rz": round(rpy[2], 3)}
                    if rpy is not None and self.end_effector != "adaptive_gripper" else None
                ),
                "orientationPolicy": "per_pick_canonical_top_down" if self.end_effector == "adaptive_gripper" else "captured",
                "note": (
                    "Computer plans robot-frame coordinate targets; myCobot firmware "
                    "handles coordinate motion through send_coords and returned joint feedback drives "
                    "the digital twin."
                ),
            },
            "notes": notes,
            "safetyGate": {
                "autoExecutePhysicalRobot": False,
                "physicalConfirmToken": PHYSICAL_CONFIRM_TOKEN,
                "maxSuggestedSpeed": SPEED_TRANSIT,
                "requiresPreviewFirst": True,
                "reason": (
                    "Pick/place states use firmware coordinate moves through send_coords. "
                    "Approach/carry use angular coordinate mode; short descend/lift/lower/"
                    "retreat states use linear coordinate mode. Stop aborts between polls."
                ),
            },
        }

    def _resolve_destination(
        self,
        place: Dict[str, Any],
        bins: Dict[str, Dict[str, Any]],
        part: Dict[str, Any],
        points: Optional[Dict[str, Dict[str, Any]]] = None,
    ) -> Optional[Dict[str, Any]]:
        if place.get("binId"):
            bin_obj = bins.get(str(place["binId"]))
            if bin_obj is None:
                return None
            return {"kind": "bin", "bin": bin_obj}
        if place.get("pointId"):
            point = (points or self.taught_points).get(str(place["pointId"]))
            if point is None or "destination" not in point.get("uses", []):
                return None
            tcp_position = point["tcpPoseM"]["position"]
            return {
                "kind": "point",
                "pointId": point["id"],
                "label": point["label"],
                "position": {
                    "x": float(tcp_position["x"]),
                    "y": float(tcp_position["y"]),
                    "z": float(point.get("supportSurfaceZ") or 0.0),
                },
            }
        if place.get("position"):
            return {"kind": "point", "position": dict(place["position"])}
        return None

    def _taught_point_motion(
        self,
        point: Dict[str, Any],
        sequence: int,
        previous_point: Optional[Dict[str, Any]] = None,
        motion_type: str = "joint",
        speed: int = SPEED_TRANSIT,
    ) -> Dict[str, Any]:
        if str(point.get("endEffector")) != self.end_effector:
            return {
                "ok": False,
                "error": (
                    f"{point.get('label')} was captured with {point.get('endEffector')}; "
                    f"select that tool or recapture it with {self.end_effector}."
                ),
            }
        if str(point.get("toolCalibrationFingerprint")) != self.tool_calibration_fingerprint():
            return {
                "ok": False,
                "error": f"{point.get('label')}'s tool calibration changed; recapture the point.",
            }
        tcp_pose = point.get("tcpPoseM") or {}
        target_position = tcp_pose.get("position") or {}
        flange_coords = [float(value) for value in point.get("firmwareFlangeCoordsMmDeg") or []]
        preferred = [float(value) for value in point.get("jointAnglesDeg") or []]
        if len(flange_coords) != 6 or len(preferred) != 6:
            return {"ok": False, "error": f"{point.get('label')} has incomplete captured robot data."}
        target = (
            float(target_position["x"]), float(target_position["y"]), float(target_position["z"]),
        )
        rpy = flange_coords[3:6]
        prefix = f"seq{sequence:02d}"
        steps: List[Dict[str, Any]] = []
        command_speed = int(clamp(speed, 1, 100))
        motion_type = str(motion_type).lower()
        if motion_type not in ("joint", "linear", "legacy"):
            motion_type = "joint"

        def fixed_step(
            suffix: str,
            name: str,
            tcp_target: Tuple[float, float, float],
            mode: int,
            previous: Optional[Tuple[float, float, float]],
            exact: bool = False,
        ) -> Dict[str, Any]:
            step = self._coordinate_step(
                f"{prefix}_{suffix}", name, tcp_target, command_speed,
                mode, rpy, "taught_point", previous=previous,
            )
            step.update({
                "pointId": point["id"],
                "pointLabel": point["label"],
                "orientationPolicy": "fixed_taught_pose",
                "preferredJointSeedDeg": [round(value, 4) for value in preferred],
                "capturedToolCalibrationFingerprint": point.get("toolCalibrationFingerprint"),
            })
            if exact:
                step["coordsMm"] = [round(value, 6) for value in flange_coords]
                step["targetFlangePoseM"] = deepcopy(point.get("flangePoseM"))
            return step

        previous_tcp: Optional[Tuple[float, float, float]] = None
        if (
            motion_type == "legacy"
            and previous_point is not None
            and previous_point.get("supportSurfaceZ") is not None
        ):
            previous_position = ((previous_point.get("tcpPoseM") or {}).get("position") or {})
            previous_target = (
                float(previous_position.get("x", 0.0)),
                float(previous_position.get("y", 0.0)),
                float(previous_position.get("z", 0.0)),
            )
            previous_clearance = (
                previous_target[0], previous_target[1],
                max(previous_target[2] + PREGRASP_RISE_M, MIN_TRANSIT_Z),
            )
            previous_coords = [
                float(value) for value in previous_point.get("firmwareFlangeCoordsMmDeg") or []
            ]
            if len(previous_coords) != 6:
                return {"ok": False, "error": f"{previous_point.get('label')} has incomplete captured robot data."}
            retreat = self._coordinate_step(
                f"{prefix}_s1_depart", "point_retreat", previous_clearance, SPEED_LIFT,
                COORD_MODE_LINEAR, previous_coords[3:6], "taught_point", previous=previous_target,
            )
            retreat.update({
                "pointId": previous_point["id"],
                "pointLabel": previous_point["label"],
                "orientationPolicy": "fixed_taught_pose",
                "preferredJointSeedDeg": deepcopy(previous_point.get("jointAnglesDeg") or []),
            })
            steps.append(retreat)
            previous_tcp = previous_clearance

        support_z = point.get("supportSurfaceZ")
        if motion_type == "joint":
            steps.append({
                "stateId": f"{prefix}_s1_arrive",
                "name": "joint_move",
                "robotCommand": "joint_move",
                "jointTargetDeg": [round(value, 6) for value in preferred],
                "capturedFlangeCoordsMmDeg": [round(value, 6) for value in flange_coords],
                "previewAngles": [round(value, 3) for value in preferred],
                "pointId": point.get("id"),
                "pointLabel": point.get("label"),
                "targetTcpPoseM": {
                    "x": target[0], "y": target[1], "z": target[2],
                },
                "durationMs": 3000,
                "speed": command_speed,
                "activeTool": self.end_effector,
                "capturedToolCalibrationFingerprint": point.get("toolCalibrationFingerprint"),
            })
        elif motion_type == "linear":
            steps.append(fixed_step("s1_arrive", "linear_move", target, COORD_MODE_LINEAR, None, exact=True))
        elif support_z is not None and target[2] <= float(support_z) + 0.10:
            approach = (target[0], target[1], max(target[2] + PREGRASP_RISE_M, MIN_TRANSIT_Z))
            steps.append(fixed_step("s2_approach", "point_approach", approach, COORD_MODE_ANGULAR, previous_tcp))
            steps.append(fixed_step("s3_arrive", "move_to_point", target, COORD_MODE_LINEAR, approach, exact=True))
        else:
            steps.append(fixed_step("s1_arrive", "move_to_point", target, COORD_MODE_ANGULAR, previous_tcp, exact=True))
        return {
            "ok": True,
            "steps": steps,
            "notes": [
                f"{'Joint' if motion_type == 'joint' else 'Linear'} move to {point.get('label') or 'embedded waypoint'}."
            ],
        }

    def _pick_height_bias_m(self) -> float:
        return clamp((self.coordinate_planner or {}).get("pickHeightBiasM", 0.0), -0.008, 0.008)

    def _minimum_table_clearance_m(self) -> float:
        return clamp(
            (self.coordinate_planner or {}).get("minimumTableClearanceM", DEFAULT_TABLE_CLEARANCE_M),
            0.002,
            0.012,
        )

    def minimum_adaptive_gripper_jaw_z(self) -> float:
        """Lowest table-safe jaw-center height for the modeled fingers."""
        return TABLE_Z + ADAPTIVE_GRIPPER_FINGER_CONTACT_LENGTH_M + self._minimum_table_clearance_m()

    def _grasp_height_model(self, part: Dict[str, Any]) -> Dict[str, float]:
        """Return one explicit vertical-contact model for a top-down pinch.

        The jaw-center TCP targets the object's vertical center.  For objects
        shorter than the finger contact section, it is raised only enough to
        keep the physical fingertip low point above the table.  Pick Z Bias is
        a bounded operator trim applied to that center target; it is not a
        hidden tool or camera offset.
        """
        position = part["position"]
        size = part["size"]
        object_center_z = float(position["z"])
        object_height = max(0.0, float(size["z"]))
        object_bottom_z = object_center_z - object_height / 2.0
        object_top_z = object_center_z + object_height / 2.0
        bias = self._pick_height_bias_m()
        minimum_table_clearance = self._minimum_table_clearance_m()
        unclamped_jaw_z = object_center_z + bias
        minimum_jaw_z = self.minimum_adaptive_gripper_jaw_z()
        jaw_center_z = max(unclamped_jaw_z, minimum_jaw_z)
        fingertip_low_z = jaw_center_z - ADAPTIVE_GRIPPER_FINGER_CONTACT_LENGTH_M
        overlap_low_z = max(object_bottom_z, fingertip_low_z)
        overlap_high_z = min(object_top_z, jaw_center_z)
        actual_overlap = max(0.0, overlap_high_z - overlap_low_z)
        desired_overlap = min(
            ADAPTIVE_GRIPPER_FINGER_CONTACT_LENGTH_M,
            max(0.0, object_top_z - max(object_bottom_z, TABLE_Z + minimum_table_clearance)),
        )
        return {
            "objectBottomZ": object_bottom_z,
            "objectCenterZ": object_center_z,
            "objectTopZ": object_top_z,
            "fingerContactLengthM": ADAPTIVE_GRIPPER_FINGER_CONTACT_LENGTH_M,
            "desiredFingerOverlapM": desired_overlap,
            "actualFingerOverlapM": actual_overlap,
            "unclampedJawCenterZ": unclamped_jaw_z,
            "jawCenterTargetZ": jaw_center_z,
            "fingertipLowTargetZ": fingertip_low_z,
            "tableClearanceM": fingertip_low_z - TABLE_Z,
            "minimumTableClearanceM": minimum_table_clearance,
            "pickHeightBiasM": bias,
        }

    def _configured_tool_tcp_offset(self) -> Dict[str, float]:
        offsets = (self.coordinate_planner or {}).get("toolOffsetsM") or TOOL_TCP_OFFSETS_M
        raw = offsets.get(self.end_effector) or TOOL_TCP_OFFSETS_M["adaptive_gripper"]
        return {
            "x": float(raw.get("x", 0.0)),
            "y": float(raw.get("y", 0.0)),
            "z": float(raw.get("z", 0.0)),
        }

    def _active_tool_profile(self) -> Dict[str, Any]:
        profiles = (self.coordinate_planner or {}).get("toolProfiles") or {}
        return deepcopy(profiles.get(self.end_effector) or DEFAULT_TOOL_PROFILES[self.end_effector])

    def _tool_correction_tuple(self) -> Tuple[float, float, float]:
        correction = self._active_tool_profile().get("tcpCorrectionLocalM") or {}
        return tuple(float(correction.get(axis, 0.0)) for axis in ("x", "y", "z"))  # type: ignore[return-value]

    def _suction_contact_distance_m(self) -> float:
        geometry = self._active_tool_profile().get("geometry") or {}
        return float(geometry.get("flangeToContactM", 0.072))

    def _coordinate_tool_rpy(self) -> Optional[List[float]]:
        config = self.coordinate_planner or {}
        rpy = config.get("toolRpyDeg")
        if isinstance(rpy, dict):
            try:
                return [float(rpy["rx"]), float(rpy["ry"]), float(rpy["rz"])]
            except (KeyError, TypeError, ValueError):
                return None
        if isinstance(rpy, list) and len(rpy) >= 3:
            try:
                return [float(rpy[0]), float(rpy[1]), float(rpy[2])]
            except (TypeError, ValueError):
                return None
        return None

    @staticmethod
    def _pose_dict(point: Tuple[float, float, float]) -> Dict[str, float]:
        return {"x": round(point[0], 4), "y": round(point[1], 4), "z": round(point[2], 4)}

    @staticmethod
    def _coords_mm(point: Tuple[float, float, float], rpy: Optional[List[float]]) -> List[Optional[float]]:
        coords: List[Optional[float]] = [
            round(float(point[0]) * 1000.0, 2),
            round(float(point[1]) * 1000.0, 2),
            round(float(point[2]) * 1000.0, 2),
        ]
        if rpy is None:
            coords.extend([None, None, None])
        else:
            coords.extend(round(float(value), 3) for value in rpy[:3])
        return coords

    @staticmethod
    def _rotate_xyz_deg(vector: Tuple[float, float, float], rpy: List[float]) -> Tuple[float, float, float]:
        x, y, z = vector
        rx, ry, rz = (math.radians(float(value)) for value in rpy[:3])
        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)

        y, z = y * cx - z * sx, y * sx + z * cx
        x, z = x * cy + z * sy, -x * sy + z * cy
        x, y = x * cz - y * sz, x * sz + y * cz
        return (x, y, z)

    def _tcp_to_flange_point(
        self,
        tcp_point: Tuple[float, float, float],
        rpy: Optional[List[float]],
    ) -> Tuple[float, float, float]:
        if rpy is not None:
            flange_rotation = rotation_from_rpy_deg(rpy)
            tcp_rotation = tcp_from_flange(
                (0.0, 0.0, 0.0), flange_rotation, self.end_effector,
                self._tool_correction_tuple(), self._suction_contact_distance_m(),
            )[1]
            return flange_from_tcp(
                tcp_point, tcp_rotation, self.end_effector,
                self._tool_correction_tuple(), self._suction_contact_distance_m(),
            )[0]
        offset = self._configured_tool_tcp_offset()
        offset_tuple = (offset["x"], offset["y"], offset["z"])
        rotated = self._rotate_xyz_deg(offset_tuple, rpy) if rpy is not None else offset_tuple
        return (
            float(tcp_point[0]) - rotated[0],
            float(tcp_point[1]) - rotated[1],
            float(tcp_point[2]) - rotated[2],
        )

    @staticmethod
    def _coord_duration_ms(
        start: Optional[Tuple[float, float, float]],
        end: Tuple[float, float, float],
        speed: int,
        minimum_ms: int = 800,
    ) -> int:
        if start is None:
            return max(minimum_ms, 1800)
        dist = math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(start, end)))
        # Firmware speed is a percentage-style value, not SI velocity. This is
        # only preview timing, so use a conservative visual estimate.
        meters_per_s = max(0.025, min(0.22, float(speed) * 0.004))
        return int(max(float(minimum_ms), dist / meters_per_s * 1000.0))

    def _coordinate_step(
        self,
        state_id: str,
        name: str,
        point: Tuple[float, float, float],
        speed: int,
        coord_mode: int,
        rpy: Optional[List[float]],
        rpy_source: str,
        previous: Optional[Tuple[float, float, float]] = None,
        gripper: Optional[str] = None,
        object_id: Optional[str] = None,
        attach_id: Optional[str] = None,
        timeout_ms: int = 14000,
    ) -> Dict[str, Any]:
        tcp_pose = self._pose_dict(point)
        flange_point = self._tcp_to_flange_point(point, rpy)
        flange_pose = self._pose_dict(flange_point)
        step: Dict[str, Any] = {
            "stateId": state_id,
            "name": name,
            "motionMode": "firmware_coords",
            "coordMode": int(coord_mode),
            "coordSpeed": int(speed),
            "coordsMm": self._coords_mm(flange_point, rpy),
            "targetTcpPoseM": tcp_pose,
            "targetFlangePoseM": flange_pose,
            "targetPoseM": tcp_pose,
            "pose": tcp_pose,
            "toolRpySource": rpy_source,
            "baseToolRpyDeg": ([round(float(value), 3) for value in rpy[:3]] if rpy is not None else None),
            "configuredToolTcpOffsetM": deepcopy(self._configured_tool_tcp_offset()),
            "activeTool": self.end_effector,
            "toolProfile": self._active_tool_profile(),
            "appliedToolCorrectionLocalM": deepcopy(
                self._active_tool_profile().get("tcpCorrectionLocalM") or {}
            ),
            "pickHeightBiasM": round(self._pick_height_bias_m(), 4),
            "coordsEstimated": rpy is None,
            "durationMs": self._coord_duration_ms(previous, point, speed),
            "timeoutMs": timeout_ms,
        }
        if gripper:
            step["gripper"] = gripper
        if object_id:
            step["objectId"] = object_id
        if attach_id:
            step["attachObjectId"] = attach_id
        return step

    @staticmethod
    def _transfer_waypoints(
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
    ) -> List[Tuple[float, float, float]]:
        """Return intermediate safe-Z points for a long angular transfer.

        The endpoints are deliberately excluded.  Use a direct subdivision
        when it clears the base; otherwise follow the shorter polar arc while
        maintaining the configured pedestal clearance.
        """
        xy_distance = math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
        start_bearing = math.degrees(math.atan2(float(start[1]), float(start[0])))
        end_bearing = math.degrees(math.atan2(float(end[1]), float(end[0])))
        signed_bearing_delta = _wrap_deg(end_bearing - start_bearing)
        bearing_delta = abs(signed_bearing_delta)
        start_radius = math.hypot(float(start[0]), float(start[1]))
        end_radius = math.hypot(float(end[0]), float(end[1]))

        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        denominator = dx * dx + dy * dy
        projection = 0.0 if denominator <= 1e-12 else max(
            0.0,
            min(1.0, -(float(start[0]) * dx + float(start[1]) * dy) / denominator),
        )
        closest_x = float(start[0]) + dx * projection
        closest_y = float(start[1]) + dy * projection
        direct_min_radius = math.hypot(closest_x, closest_y)
        route_around_base = direct_min_radius < BASE_TRANSFER_CLEARANCE_RADIUS_M
        # When routing around the base, prefer only one or two outside points
        # instead of tracing a many-point rounded arc.  The 55-degree bearing
        # cap gives IK enough branch overlap for extreme side-to-side moves.
        path_length = xy_distance
        leg_count = max(
            1,
            2 if route_around_base else int(math.ceil(path_length / MAX_TRANSFER_XY_LEG_M)),
            int(math.ceil(bearing_delta / MAX_TRANSFER_BEARING_STEP_DEG)),
        )
        safe_z = max(float(start[2]), float(end[2]))
        waypoints = []
        for index in range(1, leg_count):
            fraction = index / leg_count
            if route_around_base:
                bearing = math.radians(start_bearing + signed_bearing_delta * fraction)
                radius = max(
                    BASE_TRANSFER_CLEARANCE_RADIUS_M,
                    start_radius + (end_radius - start_radius) * fraction,
                )
                x, y = math.cos(bearing) * radius, math.sin(bearing) * radius
            else:
                x = float(start[0]) + dx * fraction
                y = float(start[1]) + dy * fraction
            waypoints.append((x, y, safe_z))
        return waypoints

    def _select_coordinate_grasp(self, part: Dict[str, Any]) -> Dict[str, Any]:
        options = self.surface_grasp_candidates(part)
        if not options:
            return self.choose_grasp(part)

        reference_jaw_yaw = -90.0
        captured = self._coordinate_tool_rpy()
        if captured is not None:
            captured_axes = tool_axis_diagnostics(rotation_from_rpy_deg(captured))
            if float(captured_axes["approachTiltDeg"]) <= 10.0:
                reference_jaw_yaw = float(captured_axes["jawYawDeg"])

        def score(option: Dict[str, Any]) -> Tuple[float, float, float]:
            width = float(option.get("objectWidthM") or 999.0)
            penalty = float(option.get("surfacePenalty") or 0.0)
            yaw_delta = abs(_wrap_deg(float(option.get("yawDeg") or 0.0) - reference_jaw_yaw))
            # A vertical pinch has no approach face: choose the narrowest
            # object axis, then the equivalent direction requiring the least
            # wrist rotation from the top-down reference.
            return (width, penalty, yaw_delta)

        return min(options, key=score)

    @staticmethod
    def _pickup_profile(part: Dict[str, Any], tool_id: str) -> Dict[str, Any]:
        profiles = part.get("pickupProfiles") or DEFAULT_PICKUP_PROFILES
        return deepcopy(profiles.get(tool_id) or DEFAULT_PICKUP_PROFILES[tool_id])

    @staticmethod
    def _object_local_offset_world(
        part: Dict[str, Any], offset: Dict[str, Any]
    ) -> Tuple[float, float, float]:
        yaw = math.radians(float(part.get("orientationDeg") or 0.0))
        local_x = float(offset.get("x", 0.0))
        local_y = float(offset.get("y", 0.0))
        return (
            math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            math.sin(yaw) * local_x + math.cos(yaw) * local_y,
            float(offset.get("z", 0.0)),
        )

    def _suction_grasp(self, part: Dict[str, Any]) -> Dict[str, Any]:
        profile = self._pickup_profile(part, "suction_gripper")
        offset = profile.get("offsetLocalM") or {}
        world_offset = self._object_local_offset_world(part, offset)
        size = part["size"]
        radius = float((self._active_tool_profile().get("geometry") or {}).get("cupDiameterM", 0.022)) / 2.0
        local_x = float(offset.get("x", 0.0))
        local_y = float(offset.get("y", 0.0))
        if abs(local_x) + radius > float(size["x"]) / 2.0 + 1e-9 or abs(local_y) + radius > float(size["y"]) / 2.0 + 1e-9:
            return {
                "ok": False,
                "error": (
                    f"Suction point is outside {part['label']}'s usable top face: the complete "
                    f"{radius * 2000.0:.0f} mm cup must remain inside the footprint."
                ),
            }
        top_z = float(part["position"]["z"]) + float(size["z"]) / 2.0
        preload = clamp(profile.get("contactPreloadM", 0.002), 0.0, 0.008)
        point = (
            float(part["position"]["x"]) + world_offset[0],
            float(part["position"]["y"]) + world_offset[1],
            top_z - preload + world_offset[2],
        )
        return {
            "ok": True,
            "strategy": "suction_top_surface",
            "graspPoint": point,
            "yawDeg": None,
            "objectWidthM": min(float(size["x"]), float(size["y"])),
            "axis": "top_surface",
            "objectCenter": deepcopy(part["position"]),
            "objectSize": deepcopy(size),
            "contactPreloadM": preload,
            "cupDiameterM": radius * 2.0,
            "pickupOffsetLocalM": deepcopy(offset),
            "tagCenter": deepcopy(part.get("tagCenter") or part.get("position")),
            "notes": [
                f"{part['label']}: suction contact at the top surface with {preload * 1000.0:.1f} mm compliant preload."
            ],
        }

    def _plan_single_pick_coordinate(
        self,
        part: Dict[str, Any],
        destination: Dict[str, Any],
        sequence: int,
        rpy: Optional[List[float]],
        rpy_source: str,
        route_low_approach: bool = False,
    ) -> Dict[str, Any]:
        prefix = f"seq{sequence:02d}"
        notes: List[str] = []
        position = part["position"]
        size = part["size"]
        half_z = float(size["z"]) / 2.0
        top_z = float(position["z"]) + half_z

        grasp_plan = (
            self._suction_grasp(part)
            if self.end_effector == "suction_gripper"
            else self._select_coordinate_grasp(part)
        )
        if grasp_plan.get("ok") is False:
            return {"ok": False, "error": grasp_plan.get("error") or "Invalid pickup setup."}
        notes.extend(grasp_plan.get("notes", []))
        grasp = tuple(float(v) for v in grasp_plan["graspPoint"])
        if self.end_effector == "adaptive_gripper":
            pickup_profile = self._pickup_profile(part, "adaptive_gripper")
            pickup_offset = self._object_local_offset_world(
                part, pickup_profile.get("offsetLocalM") or {}
            )
            grasp = tuple(grasp[index] + pickup_offset[index] for index in range(3))
            jaw_yaw = float(grasp_plan.get("yawDeg") or 0.0)
            if pickup_profile.get("jawYawOverrideDeg") is not None:
                jaw_yaw = _wrap_deg(
                    float(part.get("orientationDeg") or 0.0)
                    + float(pickup_profile["jawYawOverrideDeg"])
                )
            _, flange_rotation = top_down_flange_pose(
                grasp, jaw_yaw, self.end_effector,
                self._tool_correction_tuple(), self._suction_contact_distance_m(),
            )
            rpy = [float(value) for value in rpy_deg_from_rotation(flange_rotation)]
            rpy_source = "canonical_top_down"
            tool_axes = tool_axis_diagnostics(flange_rotation, self.end_effector)
            if not tool_axes["topDown"]:
                return {"ok": False, "error": f"canonical tool pose is not top-down ({tool_axes['approachTiltDeg']:.2f} deg)"}
        else:
            jaw_yaw = None
            # Round cups are yaw symmetric. Preview will evaluate equivalent
            # wrist yaws and retain the smallest complete-path joint travel.
            radial_yaw = math.degrees(math.atan2(grasp[1], grasp[0]))
            _, flange_rotation = top_down_flange_pose(
                grasp, radial_yaw, self.end_effector,
                self._tool_correction_tuple(), self._suction_contact_distance_m(),
            )
            rpy = [float(value) for value in rpy_deg_from_rotation(flange_rotation)]
            rpy_source = "canonical_top_down_suction"
            tool_axes = tool_axis_diagnostics(flange_rotation, self.end_effector)
        pregrasp = (
            grasp[0],
            grasp[1],
            max(float(grasp[2]) + PREGRASP_RISE_M, top_z + PREGRASP_RISE_M),
        )
        grasp_z = float(grasp[2])
        minimum_jaw_z = self.minimum_adaptive_gripper_jaw_z() if self.end_effector == "adaptive_gripper" else TABLE_Z
        if self.end_effector == "adaptive_gripper" and grasp_z <= minimum_jaw_z + 1e-6:
            notes.append(
                f"{part['label']}: pick height clamped to safe gripper pocket z {minimum_jaw_z:.3f} m."
            )
        if rpy is None:
            notes.append(
                "No saved coordinate tool orientation; capture tool RPY before running this plan on the physical robot."
            )
        if grasp_plan.get("strategy") == "suction_top_surface":
            notes.append(
                f"{part['label']}: centered suction pickup at z {grasp_z:.3f} m; side-pinch height rules are disabled."
            )
        elif grasp_plan.get("strategy") == "surface_grasp":
            notes.append(
                f"{part['label']}: coordinate centered pinch using face {grasp_plan.get('surfaceFace')}, "
                f"width {float(grasp_plan.get('objectWidthM') or 0.0):.3f} m, "
                f"height {grasp_z:.3f} m."
            )
        else:
            notes.append(f"{part['label']}: coordinate centered grip at height {grasp_z:.3f} m.")

        if destination["kind"] == "bin":
            bin_obj = destination["bin"]
            geometry = self.bin_geometry(bin_obj)
            place_xy = self.reachable_bin_drop_xy(bin_obj, geometry, size)
            center = geometry["dropCenter"]
            shift = math.hypot(float(place_xy["x"]) - float(center["x"]), float(place_xy["y"]) - float(center["y"]))
            if shift > 0.001:
                notes.append(
                    f"{part['label']}: bin drop shifted {shift:.3f} m toward the base while preserving wall clearance."
                )
            resting_z = geometry["floorZ"] + half_z
            if self.end_effector == "suction_gripper":
                preload = float(grasp_plan.get("contactPreloadM") or 0.002)
                release_z = geometry["floorZ"] + float(size["z"]) - preload
            else:
                release_z = max(
                    geometry["wallTopZ"] + half_z + BIN_RELEASE_CLEARANCE_M,
                    minimum_jaw_z,
                )
            if self.end_effector == "adaptive_gripper" and geometry["wallTopZ"] + half_z + BIN_RELEASE_CLEARANCE_M < minimum_jaw_z:
                notes.append(
                    f"{part['label']}: bin drop height clamped to safe gripper pocket z {minimum_jaw_z:.3f} m."
                )
            above_place_z = release_z + BIN_APPROACH_CLEARANCE_M
            wall_top = geometry["wallTopZ"]
            destination_info = {
                "mode": "bin",
                "id": bin_obj["id"],
                "label": bin_obj["label"],
                "position": {"x": place_xy["x"], "y": place_xy["y"], "z": release_z},
            }
        else:
            point = destination["position"]
            place_xy = {"x": float(point["x"]), "y": float(point["y"])}
            if self.end_effector == "suction_gripper":
                preload = float(grasp_plan.get("contactPreloadM") or 0.002)
                resting_z = float(point.get("z", 0.0)) + half_z
                release_z = float(point.get("z", 0.0)) + float(size["z"]) - preload
            else:
                release_z = max(float(point.get("z", 0.0)) + half_z + RELEASE_DROP_GAP_M, minimum_jaw_z)
                resting_z = max(half_z, release_z - RELEASE_DROP_GAP_M)
            if self.end_effector == "adaptive_gripper" and float(point.get("z", 0.0)) + half_z + RELEASE_DROP_GAP_M < minimum_jaw_z:
                notes.append(
                    f"{part['label']}: point drop height clamped to safe gripper pocket z {minimum_jaw_z:.3f} m."
                )
            above_place_z = release_z + PREGRASP_RISE_M
            wall_top = 0.0
            destination_info = {
                "mode": "point",
                "id": destination.get("pointId") or "manual",
                "label": destination.get("label") or "Point",
                "position": {"x": place_xy["x"], "y": place_xy["y"], "z": release_z},
            }

        # Pick clearance and place/carry clearance are independent. Forcing a
        # far-edge pick up to the bin's higher transit Z made a vertical pose
        # unreachable and encouraged a sideways wrist solution.
        pick_approach_z = pregrasp[2]
        approach_point = (grasp[0], grasp[1], pick_approach_z)
        lift_top = approach_point
        place_point = (float(place_xy["x"]), float(place_xy["y"]), release_z)
        if self.end_effector == "suction_gripper":
            # A suction TCP is the top contact point, not the object's center.
            # Keep the complete carried object above the bin wall before the
            # vertical lower; reusing the side-pinch half-height clearance can
            # let the lower half of a tall box clip the rim.
            preload = float(grasp_plan.get("contactPreloadM") or 0.002)
            carried_depth_below_tcp = max(0.0, float(size["z"]) - preload)
            place_approach_z = max(
                above_place_z,
                wall_top + carried_depth_below_tcp + TRANSIT_EXTRA_CLEARANCE_M,
            )
        else:
            place_approach_z = max(
                above_place_z,
                wall_top + half_z + TRANSIT_EXTRA_CLEARANCE_M,
            )
        above_place = (place_point[0], place_point[1], place_approach_z)
        retreat_top = above_place
        placed_position = {"x": place_point[0], "y": place_point[1], "z": resting_z}

        part_id = part["id"]
        grasp_meta = {
            "strategy": grasp_plan["strategy"],
            "objectWidthM": grasp_plan.get("objectWidthM"),
            "yawDeg": grasp_plan.get("yawDeg"),
            "axis": grasp_plan.get("axis"),
            "graspPoint": self._pose_dict(grasp),
            "pregraspPoint": self._pose_dict(approach_point),
            "graspInsetM": grasp_plan.get("graspInsetM"),
            "surfaceFace": grasp_plan.get("surfaceFace"),
            "surfaceNormal": grasp_plan.get("surfaceNormal"),
            "surfaceGripDepthM": grasp_plan.get("surfaceGripDepthM"),
            "surfaceCenterBandRatio": grasp_plan.get("surfaceCenterBandRatio"),
            "objectCenter": grasp_plan.get("objectCenter"),
            "objectSize": grasp_plan.get("objectSize"),
            "objectBoundsZ": grasp_plan.get("objectBoundsZ"),
            "heightModel": grasp_plan.get("graspHeightModel"),
            "toolClearance": grasp_plan.get("toolClearance"),
            "captureWindow": grasp_plan.get("captureWindow"),
            "planner": "firmware_coordinate",
            "toolRpySource": rpy_source,
            "activeTool": self.end_effector,
            "tagCenter": grasp_plan.get("tagCenter") or deepcopy(part.get("position")),
            "pickupOffsetLocalM": deepcopy(
                self._pickup_profile(part, self.end_effector).get("offsetLocalM") or {}
            ),
            "globalToolCorrectionLocalM": deepcopy(
                self._active_tool_profile().get("tcpCorrectionLocalM") or {}
            ),
            "contactPreloadM": grasp_plan.get("contactPreloadM"),
            "cupDiameterM": grasp_plan.get("cupDiameterM"),
            "desiredJawYawDeg": round(jaw_yaw, 3) if jaw_yaw is not None else None,
            "toolApproachTiltDeg": round(float(tool_axes["approachTiltDeg"]), 4) if tool_axes else None,
            "toolApproachAxis": list(tool_axes["approachAxis"]) if tool_axes else None,
            "jawAxis": list(tool_axes["jawAxis"]) if tool_axes else None,
        }

        steps: List[Dict[str, Any]] = []
        staging_point: Optional[Tuple[float, float, float]] = None
        approach_radius = math.hypot(float(approach_point[0]), float(approach_point[1]))
        if (
            route_low_approach
            and float(approach_point[2]) < APPROACH_STAGING_Z_M - 0.015
            and approach_radius <= APPROACH_STAGING_MAX_RADIUS_M
        ):
            staging_point = (
                float(approach_point[0]),
                float(approach_point[1]),
                APPROACH_STAGING_Z_M,
            )
            staging_step = self._coordinate_step(
                f"{prefix}_s1_staging", "approach_staging", staging_point,
                SPEED_TRANSIT, COORD_MODE_ANGULAR, rpy, rpy_source,
                gripper="open", object_id=part_id,
            )
            staging_step.update({
                "gripperAction": "open_before_approach",
                "gripperActionTiming": "before_move",
                "grasp": grasp_meta,
                "desiredJawYawDeg": grasp_meta.get("desiredJawYawDeg"),
                "routingReason": "home_to_low_approach_continuity",
            })
            steps.append(staging_step)
            notes.append(
                f"{part['label']}: approach routed through a top-down staging point "
                "before the short vertical pickup move."
            )
        elif route_low_approach and approach_radius > APPROACH_STAGING_MAX_RADIUS_M:
            notes.append(
                f"{part['label']}: unreachable high staging point omitted at outer-workspace "
                f"radius {approach_radius:.3f} m; moving directly to the validated approach pose."
            )
        approach_step = self._coordinate_step(
            f"{prefix}_s1_approach", "approach", approach_point, SPEED_TRANSIT,
            COORD_MODE_LINEAR if staging_point else COORD_MODE_ANGULAR,
            rpy, rpy_source, previous=staging_point,
            gripper="open", object_id=part_id,
        )
        approach_step.update({
            "grasp": grasp_meta,
            "desiredJawYawDeg": grasp_meta.get("desiredJawYawDeg"),
        })
        if staging_point is None:
            approach_step.update({
                "gripperAction": "open_before_approach",
                "gripperActionTiming": "before_move",
            })
        steps.append(approach_step)

        descend_step = self._coordinate_step(
            f"{prefix}_s2_descend", "descend", grasp, SPEED_DESCEND,
            COORD_MODE_LINEAR, rpy, rpy_source, previous=approach_point,
            gripper="open", object_id=part_id, timeout_ms=9000,
        )
        descend_step["grasp"] = grasp_meta
        descend_step["desiredJawYawDeg"] = grasp_meta.get("desiredJawYawDeg")
        steps.append(descend_step)

        steps.append({
            "stateId": f"{prefix}_s3_grip",
            "name": "auto_grip",
            "gripper": "closed",
            "gripperAction": "auto_grip",
            "gripperActionTiming": "after_arrival",
            "attachObjectId": part_id,
            "objectId": part_id,
            "targetTcpPoseM": self._pose_dict(grasp),
            "targetPoseM": self._pose_dict(grasp),
            "pose": self._pose_dict(grasp),
            "grasp": grasp_meta,
            "durationMs": 1100,
        })

        lift_step = self._coordinate_step(
            f"{prefix}_s4_lift", "lift", lift_top, SPEED_LIFT,
            COORD_MODE_LINEAR, rpy, rpy_source, previous=grasp,
            gripper="closed", object_id=part_id, attach_id=part_id,
        )
        lift_step["grasp"] = grasp_meta
        steps.append(lift_step)

        transfer_waypoints = self._transfer_waypoints(lift_top, above_place)
        previous_transfer_point = lift_top
        for transfer_index, transfer_point in enumerate(transfer_waypoints, 1):
            transfer_step = self._coordinate_step(
                f"{prefix}_s5_transfer_{transfer_index}", "transfer", transfer_point,
                SPEED_TRANSIT, COORD_MODE_ANGULAR, rpy, rpy_source,
                previous=previous_transfer_point, gripper="closed", object_id=part_id,
                attach_id=part_id,
            )
            transfer_step["transferSubdivision"] = {
                "index": transfer_index,
                "count": len(transfer_waypoints),
                "reason": "joint_continuity",
            }
            steps.append(transfer_step)
            previous_transfer_point = transfer_point
        if transfer_waypoints:
            notes.append(
                f"{part['label']}: cross-table carry split into "
                f"{len(transfer_waypoints) + 1} shorter moves for joint continuity."
            )

        carry_step = self._coordinate_step(
            f"{prefix}_s5_carry", "carry", above_place, SPEED_TRANSIT,
            COORD_MODE_ANGULAR, rpy, rpy_source, previous=previous_transfer_point,
            gripper="closed", object_id=part_id, attach_id=part_id,
        )
        steps.append(carry_step)

        lower_step = self._coordinate_step(
            f"{prefix}_s6_lower", "lower", place_point, SPEED_DESCEND,
            COORD_MODE_LINEAR, rpy, rpy_source, previous=above_place,
            gripper="closed", object_id=part_id, attach_id=part_id, timeout_ms=9000,
        )
        steps.append(lower_step)

        steps.append({
            "stateId": f"{prefix}_s7_release",
            "name": "release_gripper",
            "gripper": "open",
            "gripperAction": "open_at_drop",
            "gripperActionTiming": "after_arrival",
            "releaseObjectId": part_id,
            "objectId": part_id,
            "placedPosition": placed_position,
            "targetTcpPoseM": self._pose_dict(place_point),
            "targetPoseM": self._pose_dict(place_point),
            "pose": self._pose_dict(place_point),
            "durationMs": 1000,
        })

        steps.append(self._coordinate_step(
            f"{prefix}_s8_retreat", "retreat", retreat_top, SPEED_LIFT,
            COORD_MODE_LINEAR, rpy, rpy_source, previous=place_point,
            gripper="open", object_id=part_id,
        ))

        return {
            "ok": True,
            "steps": steps,
            "notes": notes,
            "placedPosition": placed_position,
            "destination": destination_info,
        }

    def choose_grasp(self, part: Dict[str, Any]) -> Dict[str, Any]:
        candidates = self.surface_grasp_candidates(part)
        if candidates:
            return candidates[0]

        position = part["position"]
        size = part["size"]
        height_model = self._grasp_height_model(part)
        radial_yaw = math.degrees(math.atan2(float(position["y"]), float(position["x"])))
        fallback_z = height_model["jawCenterTargetZ"]
        return {
            "strategy": "centered_top_down",
            "graspPoint": (float(position["x"]), float(position["y"]), fallback_z),
            "pregraspPoint": (
                float(position["x"]),
                float(position["y"]),
                max(fallback_z + PREGRASP_RISE_M, height_model["objectTopZ"] + PREGRASP_RISE_M),
            ),
            "yawDeg": _wrap_deg(radial_yaw),
            "objectWidthM": round(max(float(size["x"]), float(size["y"])), 4),
            "axis": "radial",
            "graspHeightModel": height_model,
            "notes": [],
        }

    def surface_grasp_candidates(self, part: Dict[str, Any]) -> List[Dict[str, Any]]:
        position = part["position"]
        size = part["size"]
        part_type = str(part.get("type") or "box")
        orientation = _wrap_deg(float(part.get("orientationDeg") or 0.0))
        center_z = float(position["z"])
        half_z = float(size["z"]) / 2.0
        bottom_z = center_z - half_z
        top_z = center_z + half_z
        box_like = part_type in {"box", "open-box", "rectangle"}
        if not box_like or not bool(part.get("graspable", True)):
            return []

        local_x = float(size["x"])
        local_y = float(size["y"])
        local_z = float(size["z"])
        height_model = self._grasp_height_model(part)
        grasp_z = height_model["jawCenterTargetZ"]
        physical_clamped = abs(grasp_z - height_model["unclampedJawCenterZ"]) > 1e-6
        center_band = max(0.0, min(1.0, SURFACE_CENTER_BAND_RATIO))
        surface_defs = [
            ("+X", "local_x", local_x, local_y, orientation, 1.0),
            ("-X", "local_x", local_x, local_y, orientation, -1.0),
            ("+Y", "local_y", local_y, local_x, orientation + 90.0, 1.0),
            ("-Y", "local_y", local_y, local_x, orientation + 90.0, -1.0),
        ]
        valid_widths = [
            width for _, _, width, _, _, _ in surface_defs
            if width <= GRIPPER_MAX_SIDE_PINCH_WIDTH_M
        ]
        narrowest_width = min(valid_widths) if valid_widths else None
        candidates: List[Dict[str, Any]] = []
        rejected: List[str] = []
        for face, axis, width, face_span, yaw_raw, sign in surface_defs:
            yaw = _wrap_deg(yaw_raw if sign > 0 else yaw_raw + 180.0)
            if width > GRIPPER_MAX_SIDE_PINCH_WIDTH_M:
                rejected.append(f"{face} too wide {width:.3f} m")
                continue
            notes: List[str] = []
            if width < GRIPPER_MIN_SIDE_PINCH_WIDTH_M:
                notes.append(
                    f"{part['label']}: {face} side is very thin ({width:.3f} m); grip may be weak."
                )
            yaw_rad = math.radians(yaw)
            normal = (math.cos(yaw_rad), math.sin(yaw_rad))
            # Keep the gripper pocket centered over the part. The chosen face
            # only controls the jaw axis/yaw and the width being pinched.
            depth = 0.0
            grasp_x = float(position["x"])
            grasp_y = float(position["y"])
            corner_clearance = face_span * (1.0 - center_band) * 0.5
            width_penalty = 0.0 if narrowest_width is None else max(0.0, width - narrowest_width) * 5000.0
            surface_penalty = width_penalty
            notes.append(
                f"{part['label']}: surface side_grip candidate {face}, width {width:.3f} m, "
                f"centered XY, height {grasp_z:.3f} m, yaw {yaw:.1f} deg."
            )
            notes.append(
                f"{part['label']}: {face} jaw center z {grasp_z:.3f} m gives "
                f"{height_model['actualFingerOverlapM']:.3f} m finger overlap and "
                f"{height_model['tableClearanceM']:.3f} m table clearance."
            )
            if physical_clamped:
                notes.append(
                    f"{part['label']}: {face} grip height clamped to safe pocket z {grasp_z:.3f} m "
                    f"(object {bottom_z:.3f}-{top_z:.3f} m)."
                )
            if rejected:
                notes.append(f"{part['label']}: rejected faces: {', '.join(rejected)}.")
            candidates.append({
                "strategy": "surface_grasp",
                "graspPoint": (grasp_x, grasp_y, grasp_z),
                "pregraspPoint": (
                    grasp_x,
                    grasp_y,
                    max(grasp_z + PREGRASP_RISE_M, top_z + PREGRASP_RISE_M),
                ),
                "yawDeg": yaw,
                "graspInsetM": depth,
                "objectWidthM": round(width, 4),
                "axis": axis,
                "surfaceFace": face,
                "surfaceNormal": {"x": round(normal[0], 5), "y": round(normal[1], 5), "z": 0.0},
                "surfaceGripDepthM": round(depth, 4),
                "surfaceFaceClearanceM": 0.0,
                "surfaceCenterBandRatio": round(center_band, 3),
                "surfaceCornerClearanceM": round(corner_clearance, 4),
                "surfacePenalty": round(surface_penalty, 3),
                "targetGripperMountDeg": GRIPPER_MOUNT_NEUTRAL_J6_DEG,
                "objectCenter": {
                    "x": float(position["x"]),
                    "y": float(position["y"]),
                    "z": center_z,
                },
                "objectSize": {
                    "x": local_x,
                    "y": local_y,
                    "z": local_z,
                },
                "objectBoundsZ": {
                    "bottom": bottom_z,
                    "top": top_z,
                },
                "graspHeightModel": height_model,
                "toolClearance": {
                    "fingerLowZ": round(height_model["fingertipLowTargetZ"], 4),
                    "tableClearanceM": round(height_model["tableClearanceM"], 4),
                    "minimumTableClearanceM": height_model["minimumTableClearanceM"],
                },
                "captureWindow": {
                    "xyToleranceM": SIDE_PINCH_XY_TOLERANCE_M,
                    "zBelowPocketM": ADAPTIVE_GRIPPER_FINGER_CONTACT_LENGTH_M,
                    "zAbovePocketM": SIDE_PINCH_CAPTURE_ABOVE_POCKET_M,
                    "yawToleranceDeg": SIDE_PINCH_YAW_TOLERANCE_DEG,
                    "surfaceCenterBandRatio": round(center_band, 3),
                },
                "notes": notes,
            })
        return candidates

    # ----------------------------------------------- single-pick fallback

    def plan_pick(self, body: Dict[str, Any], robot_status: Dict[str, Any]) -> Dict[str, Any]:
        """Single pick+place plan (Realtime agent and quick tests)."""
        with self.lock:
            part = self.parts.get(str(body.get("objectId") or ""))
        if part is None:
            query = str(body.get("objectQuery") or body.get("prompt") or "part")
            part = self.match_part(query)
        if part is None:
            return {"ok": False, "error": "No parts are in the scene yet."}

        place: Dict[str, Any] = {"type": "place"}
        if body.get("binId") or body.get("destinationId"):
            place["binId"] = str(body.get("binId") or body.get("destinationId"))
        elif body.get("place"):
            place["position"] = body["place"]
        else:
            with self.lock:
                first_bin = next(iter(self.bins.values()), None)
            if first_bin is not None:
                place["binId"] = first_bin["id"]
            else:
                p = part["position"]
                place["position"] = {"x": float(p["x"]) + 0.08, "y": float(p["y"]) - 0.08, "z": 0.0}

        start = robot_status.get("lastAngles") or HOME_ANGLES
        plan = self.plan_program(
            [{"type": "pick", "objectId": part["id"]}, place],
            [float(v) for v in start],
            program_name=f"pick {part['label']}",
        )
        if plan.get("ok"):
            plan["targetObject"] = part
        return plan

    def apply_executed_steps(
        self, executed: List[Dict[str, Any]], physical_run_ok: bool = False
    ) -> None:
        """Persist successful virtual-part placements; live AprilTags remain authoritative."""
        if not physical_run_ok:
            return
        with self.lock:
            changed = False
            for step in executed or []:
                if step.get("preventSceneMove"):
                    continue
                part_id = step.get("releaseObjectId")
                placed = step.get("placedPosition")
                if part_id and placed and part_id in self.parts:
                    part = self.parts[part_id]
                    if part.get("trackingMode") == "apriltag" or part.get("source") == "camera":
                        # The next valid tag frame supplies the actual pose. A
                        # commanded drop coordinate must never replace it.
                        continue
                    part["position"] = {
                        "x": float(placed["x"]),
                        "y": float(placed["y"]),
                        "z": float(placed["z"]),
                    }
                    part["updatedAt"] = time.time()
                    changed = True
            if changed:
                self._save_locked()
