#!/usr/bin/env python3
"""Calibration-backed planar localization for the myCobot workcell."""

from __future__ import annotations

import math
import statistics
import threading
import time
from collections import deque
from copy import deepcopy
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

DICTIONARY_NAME = "DICT_APRILTAG_36h11"
DEFAULT_MARKER_SIZE_M = 0.05
MAX_CONDITION_NUMBER = 1e6
MAX_REPROJECTION_RMS_PX = 10.0
MAX_REPROJECTION_PX = 18.0
MIN_MARKERS = 3
MIN_COVERAGE_RATIO = 0.12
CAMERA_MOVE_LIMIT_M = 0.008
MAX_INTRINSIC_RMS_PX = 2.5
MAX_INTRINSIC_VIEW_ERROR_PX = 4.0
MAX_VERIFICATION_RMS_M = 0.010
MAX_VERIFICATION_ERROR_M = 0.020
MAX_STATIONARY_SPREAD_M = 0.005


def aruco_dictionary(name: str = DICTIONARY_NAME):
    value = getattr(cv2.aruco, str(name), None)
    if value is None:
        raise ValueError(f"Unknown OpenCV marker dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(value)


def decode_jpeg(jpeg: bytes):
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("Could not decode camera frame.")
    return image


def encode_jpeg(image, quality: int = 90) -> bytes:
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("Could not encode debug frame.")
    return bytes(encoded)


def apply_homography(h: np.ndarray, point: Sequence[float]) -> Tuple[float, float]:
    src = np.asarray([[[float(point[0]), float(point[1])]]], dtype=np.float64)
    dst = cv2.perspectiveTransform(src, h)[0][0]
    return float(dst[0]), float(dst[1])


def marker_robot_corners(marker: Dict[str, Any], default_size: float) -> np.ndarray:
    corners = marker.get("corners")
    if isinstance(corners, list) and len(corners) == 4:
        return np.asarray([[float(p["x"]), float(p["y"])] for p in corners], dtype=np.float64)
    center = marker.get("center") or {}
    size = float(marker.get("sizeM") or default_size)
    yaw = math.radians(float(marker.get("yawDeg") or 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    # OpenCV marker corners are top-left, top-right, bottom-right,
    # bottom-left. With +X forward and +Y left, an upright (yaw=0) marker
    # therefore starts at (+X,+Y) and proceeds clockwise in the image.
    # Positive yaw remains counterclockwise in the robot's +X/+Y plane.
    local = [(size / 2, size / 2), (size / 2, -size / 2), (-size / 2, -size / 2), (-size / 2, size / 2)]
    return np.asarray([
        [float(center.get("x", 0.0)) + c * x - s * y, float(center.get("y", 0.0)) + s * x + c * y]
        for x, y in local
    ], dtype=np.float64)


def reference_layout_errors(reference: Dict[int, Dict[str, Any]], default_size: float) -> List[str]:
    if not all(marker_id in reference for marker_id in (0, 1, 2, 3)):
        return []
    centers = {marker_id: marker_robot_corners(reference[marker_id], default_size).mean(axis=0) for marker_id in (0, 1, 2, 3)}
    errors = []
    minimum_separation = default_size
    for left in range(4):
        for right in range(left + 1, 4):
            if float(np.linalg.norm(centers[left] - centers[right])) < minimum_separation:
                errors.append(f"ids_{left}_{right}_duplicate_or_too_close")
    forward_x = min(centers[0][0], centers[1][0])
    rear_x = max(centers[2][0], centers[3][0])
    if forward_x <= rear_x:
        errors.append("ids_0_1_must_be_forward_of_2_3")
    if not (centers[0][1] > centers[1][1] and centers[3][1] > centers[2][1]):
        errors.append("ids_0_3_must_be_left_of_1_2")
    polygon = np.asarray([centers[index] for index in (0, 1, 2, 3)], np.float32)
    if abs(float(cv2.contourArea(polygon))) < default_size * default_size:
        errors.append("reference_centers_nearly_collinear")
    return errors


class CharucoCalibrationSession:
    """In-memory ChArUco sample collector; solved results are JSON serializable."""

    def __init__(self, squares_x: int = 7, squares_y: int = 5, square_m: float = 0.03, marker_m: float = 0.022) -> None:
        self.dictionary = aruco_dictionary()
        self.board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_m, marker_m, self.dictionary)
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.samples: List[Tuple[np.ndarray, np.ndarray]] = []
        self.sample_signatures: List[Tuple[float, float, float, float, float]] = []
        self.sample_quality: List[Dict[str, Any]] = []
        self.image_size: Optional[Tuple[int, int]] = None

    def clear(self) -> Dict[str, Any]:
        self.samples.clear()
        self.sample_signatures.clear()
        self.sample_quality.clear()
        self.image_size = None
        return self.status()

    def status(self) -> Dict[str, Any]:
        return {
            "sampleCount": len(self.samples),
            "imageSize": list(self.image_size) if self.image_size else None,
            "diversity": self._diversity_report(),
            "samples": list(self.sample_quality),
        }

    def remove_last(self) -> Dict[str, Any]:
        if self.samples:
            self.samples.pop()
            self.sample_signatures.pop()
            self.sample_quality.pop()
        if not self.samples:
            self.image_size = None
        return self.status()

    def capture(self, jpeg: bytes) -> Dict[str, Any]:
        image = decode_jpeg(jpeg)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Workspace tags share this dictionary and IDs 0-3 with the ChArUco
        # board. If both are visible, OpenCV sees duplicate IDs and can return
        # zero interpolated corners. Isolate the board's dense marker cluster
        # while leaving the permanent workspace tags installed.
        marker_corners, marker_ids, _ = cv2.aruco.ArucoDetector(self.dictionary).detectMarkers(gray)
        crop, offset, board_marker_count = self._board_crop(gray, marker_corners)
        charuco_corners, charuco_ids, _, _ = self.detector.detectBoard(crop)
        if charuco_corners is not None and offset != (0, 0):
            charuco_corners = np.asarray(charuco_corners, np.float32) + np.asarray(offset, np.float32)
        count = 0 if charuco_ids is None else len(charuco_ids)
        if count < 8:
            detected = 0 if marker_ids is None else len(marker_ids)
            error = (
                f"Board found, but only {count}/8 usable ChArUco corners were resolved. "
                "Move the board closer, remove glare or shadow, and keep its outside edges visible."
                if detected else
                "No ChArUco markers were found. Check that the ChArUco PDF is facing the camera, sharp, and printed at 100%."
            )
            return {"ok": False, "error": error, "cornerCount": count, "detectedMarkerCount": detected, "boardMarkerCount": board_marker_count, **self.status()}
        object_points, image_points = self.board.matchImagePoints(charuco_corners, charuco_ids)
        flat = np.asarray(image_points).reshape(-1, 2)
        minimum, maximum = flat.min(axis=0), flat.max(axis=0)
        planar_h, _ = cv2.findHomography(np.asarray(object_points).reshape(-1, 3)[:, :2], flat)
        tilt = 0.0
        if planar_h is not None:
            planar_h = planar_h / planar_h[2, 2]
            tilt = float(math.hypot(planar_h[2, 0] * 0.21, planar_h[2, 1] * 0.15))
        signature = (
            float(flat[:, 0].mean() / gray.shape[1]), float(flat[:, 1].mean() / gray.shape[0]),
            float((maximum[0] - minimum[0]) / gray.shape[1]), float((maximum[1] - minimum[1]) / gray.shape[0]),
            tilt,
        )
        coverage = signature[2] * signature[3]
        if coverage < 0.025:
            return {"ok": False, "error": "ChArUco board covers too little of the frame.", "cornerCount": count, **self.status()}
        if self.sample_signatures and min(math.sqrt(sum((a - b) ** 2 for a, b in zip(signature, prior))) for prior in self.sample_signatures) < 0.025:
            return {"ok": False, "error": "Sample is too similar to an existing view; move or tilt the board.", "cornerCount": count, **self.status()}
        self.samples.append((np.asarray(object_points, np.float32), np.asarray(image_points, np.float32)))
        self.sample_signatures.append(signature)
        self.sample_quality.append({
            "index": len(self.samples), "centerX": signature[0], "centerY": signature[1],
            "coverageRatio": coverage, "tiltScore": tilt, "cornerCount": count,
        })
        self.image_size = (gray.shape[1], gray.shape[0])
        return {"ok": True, "cornerCount": count, "markerCount": board_marker_count, "detectedMarkerCount": 0 if marker_ids is None else len(marker_ids), **self.status()}

    @staticmethod
    def _board_crop(gray: np.ndarray, marker_corners: Sequence[np.ndarray]) -> Tuple[np.ndarray, Tuple[int, int], int]:
        if len(marker_corners) < 4:
            return gray, (0, 0), len(marker_corners)
        centers = np.asarray([np.asarray(corner).reshape(4, 2).mean(axis=0) for corner in marker_corners])
        sides = np.asarray([
            np.mean(np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1))
            for points in (np.asarray(corner).reshape(4, 2) for corner in marker_corners)
        ])
        adjacency = np.linalg.norm(centers[:, None] - centers[None, :], axis=2) < max(12.0, float(np.median(sides)) * 3.2)
        seen, components = set(), []
        for start in range(len(marker_corners)):
            if start in seen:
                continue
            stack, component = [start], []
            seen.add(start)
            while stack:
                current = stack.pop()
                component.append(current)
                for neighbor in np.where(adjacency[current])[0]:
                    neighbor = int(neighbor)
                    if neighbor not in seen:
                        seen.add(neighbor)
                        stack.append(neighbor)
            components.append(component)
        board = max(components, key=len)
        if len(board) < 4:
            return gray, (0, 0), len(board)
        points = np.concatenate([np.asarray(marker_corners[index]).reshape(-1, 2) for index in board])
        padding = max(12, int(np.median(sides[board]) * 1.5))
        x0, y0 = np.maximum(0, np.floor(points.min(axis=0) - padding)).astype(int)
        x1, y1 = np.minimum([gray.shape[1], gray.shape[0]], np.ceil(points.max(axis=0) + padding)).astype(int)
        return gray[y0:y1, x0:x1], (int(x0), int(y0)), len(board)

    def _diversity_report(self) -> Dict[str, Any]:
        signatures = self.sample_signatures
        centers = [(item[0], item[1]) for item in signatures]
        quadrants = {
            "upperLeft": any(x < 0.5 and y < 0.5 for x, y in centers),
            "upperRight": any(x >= 0.5 and y < 0.5 for x, y in centers),
            "lowerLeft": any(x < 0.5 and y >= 0.5 for x, y in centers),
            "lowerRight": any(x >= 0.5 and y >= 0.5 for x, y in centers),
        }
        center = any(0.35 <= x <= 0.65 and 0.35 <= y <= 0.65 for x, y in centers)
        scales = set()
        for item in signatures:
            area = item[2] * item[3]
            scales.add("small" if area < 0.08 else "medium" if area < 0.18 else "large")
        tilted = sum(1 for item in signatures if item[4] >= 0.08)
        missing = []
        if not center:
            missing.append("center")
        missing.extend(name for name, present in quadrants.items() if not present)
        # Two clearly different apparent sizes are enough to constrain the
        # practical lens model. Requiring all three coarse bins made the
        # largest size effectively mandatory even though these bins use the
        # inner ChArUco corners rather than the printed board boundary.
        if len(scales) < 2:
            missing.append("second_board_scale")
        if tilted < 4:
            missing.append("four_tilted_views")
        return {
            "centerCovered": center, "regions": quadrants, "scaleLevels": sorted(scales),
            "scaleLevelCount": len(scales),
            "tiltedViewCount": tilted, "missing": missing, "passed": not missing,
        }

    def solve(self, minimum_samples: int = 12) -> Dict[str, Any]:
        if len(self.samples) < minimum_samples or self.image_size is None:
            return {"ok": False, "error": f"At least {minimum_samples} valid samples are required.", **self.status()}
        diversity = self._diversity_report()
        if not diversity["passed"]:
            return {"ok": False, "error": "Calibration views lack required position, scale, or tilt diversity.", **self.status()}
        object_points = [sample[0] for sample in self.samples]
        image_points = [sample[1] for sample in self.samples]
        extended = cv2.calibrateCameraExtended(object_points, image_points, self.image_size, None, None)
        rms, matrix, distortion, rvecs, tvecs, _, _, per_view = extended
        view_errors = [float(value) for value in np.asarray(per_view).reshape(-1)]
        quality_ok = bool(
            math.isfinite(rms) and float(rms) <= MAX_INTRINSIC_RMS_PX and
            view_errors and max(view_errors) <= MAX_INTRINSIC_VIEW_ERROR_PX
        )
        result = {
            "ok": quality_ok,
            "cameraMatrix": matrix.tolist(),
            "distortionCoefficients": distortion.reshape(-1).tolist(),
            "imageSize": {"width": self.image_size[0], "height": self.image_size[1]},
            "intrinsicRmsPx": float(rms),
            "perViewErrorsPx": view_errors,
            "maximumViewErrorPx": max(view_errors) if view_errors else None,
            "diversity": diversity,
            "sampleCount": len(self.samples),
            "calibratedAt": time.time(),
        }
        if not quality_ok:
            result["error"] = (
                f"Intrinsic calibration exceeds limits: RMS {float(rms):.3f} px (limit {MAX_INTRINSIC_RMS_PX:.1f}), "
                f"worst view {max(view_errors):.3f} px (limit {MAX_INTRINSIC_VIEW_ERROR_PX:.1f})."
            )
        return result


class TrackStabilizer:
    """Three-frame median filter for deterministic AprilTag identities."""

    def __init__(self, window: int = 3, stale_s: float = 35.0) -> None:
        self.window = window
        self.stale_s = stale_s
        self.tracks: Dict[str, Dict[str, Any]] = {}

    @staticmethod
    def _angle_delta_deg(a: float, b: float) -> float:
        return ((float(a) - float(b) + 180.0) % 360.0) - 180.0

    def update(self, detections: List[Dict[str, Any]], now: Optional[float] = None) -> List[Dict[str, Any]]:
        now = float(now or time.time())
        for detection in detections:
            track_id = str(detection.get("trackId") or detection.get("id") or "")
            if not track_id:
                continue
            track = self.tracks.get(track_id)
            if track is None or now - float(track.get("updatedAt") or 0.0) > 1.0:
                track = {
                    "points": deque(maxlen=self.window), "updatedAt": now,
                    "angles": deque(maxlen=self.window),
                    "heights": deque(maxlen=5),
                    "committed": None, "committedAngleDeg": None,
                    "jumpCandidate": None, "jumpCount": 0,
                }
                self.tracks[track_id] = track
            observed = (float(detection["position"]["x"]), float(detection["position"]["y"]))
            track["points"].append(observed)
            track["angles"].append(math.radians(float(detection.get("orientationDeg") or 0.0)))
            track["heights"].append(float(detection["position"].get("z", 0.0)))
            track["updatedAt"] = now
            xs = [p[0] for p in track["points"]]
            ys = [p[1] for p in track["points"]]
            median = (statistics.median(xs), statistics.median(ys))
            detection["trackId"] = track_id
            detection["id"] = track_id
            median_z = statistics.median(track["heights"])
            detection["observedPosition"] = {"x": median[0], "y": median[1], "z": median_z}
            committed = track.get("committed")
            if committed is None:
                committed = median
            distance = math.hypot(median[0] - committed[0], median[1] - committed[1])
            observed_distance = math.hypot(observed[0] - committed[0], observed[1] - committed[1])
            if observed_distance > 0.025:
                candidate = track.get("jumpCandidate")
                if candidate and math.hypot(observed[0] - candidate[0], observed[1] - candidate[1]) <= 0.004:
                    track["jumpCount"] += 1
                else:
                    track["jumpCandidate"], track["jumpCount"] = observed, 1
                next_committed = median if track["jumpCount"] >= 2 else committed
                if track["jumpCount"] >= 2:
                    track["jumpCandidate"], track["jumpCount"] = None, 0
            elif distance <= 0.0015:
                next_committed = committed
                track["jumpCandidate"], track["jumpCount"] = None, 0
            else:
                next_committed = median
                track["jumpCandidate"], track["jumpCount"] = None, 0
            track["committed"] = next_committed
            detection["committedPosition"] = {"x": next_committed[0], "y": next_committed[1], "z": median_z}
            detection["position"]["x"], detection["position"]["y"] = next_committed
            detection["position"]["z"] = median_z
            sin_mean = statistics.mean(math.sin(value) for value in track["angles"])
            cos_mean = statistics.mean(math.cos(value) for value in track["angles"])
            observed_angle = math.degrees(math.atan2(sin_mean, cos_mean))
            committed_angle = track.get("committedAngleDeg")
            if committed_angle is None or abs(self._angle_delta_deg(observed_angle, committed_angle)) > 1.0:
                committed_angle = observed_angle
            track["committedAngleDeg"] = committed_angle
            detection["orientationDeg"] = committed_angle
            detection["stabilizationProgress"] = {
                "jumpFrames": int(track.get("jumpCount") or 0), "jumpFramesRequired": 2,
            }
            detection["stabilitySpreadM"] = max(max(xs) - min(xs), max(ys) - min(ys)) if len(xs) > 1 else 0.0
        for track_id in list(self.tracks):
            if now - float(self.tracks[track_id]["updatedAt"]) > self.stale_s:
                del self.tracks[track_id]
        return detections


class FiducialLocalizer:
    def __init__(self) -> None:
        self.detector_cache: Dict[str, Any] = {}
        self.stabilizer = TrackStabilizer()
        self.surface_tracks: Dict[str, Dict[str, Any]] = {}
        self.last_object_tag_rejections: List[Dict[str, Any]] = []
        self.last_object_tag_statuses: Dict[int, Dict[str, Any]] = {}

    def _detector(self, dictionary_name: str):
        if dictionary_name not in self.detector_cache:
            self.detector_cache[dictionary_name] = cv2.aruco.ArucoDetector(aruco_dictionary(dictionary_name))
        return self.detector_cache[dictionary_name]

    @staticmethod
    def _intrinsics(calibration: Dict[str, Any]):
        intrinsics = calibration.get("intrinsics") or {}
        matrix = intrinsics.get("cameraMatrix")
        distortion = intrinsics.get("distortionCoefficients")
        if not matrix or distortion is None:
            return None, None
        return np.asarray(matrix, np.float64), np.asarray(distortion, np.float64)

    def localize(self, jpeg: bytes, camera_config: Dict[str, Any], calibration: Dict[str, Any], draw_debug: bool = True,
                 registered_parts: Optional[List[Dict[str, Any]]] = None,
                 registered_bins: Optional[List[Dict[str, Any]]] = None,
                 support_surfaces: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
        image = decode_jpeg(jpeg)
        height, width = image.shape[:2]
        matrix, distortion = self._intrinsics(calibration)
        if matrix is None:
            return {"ok": False, "error": "camera_intrinsics_missing", "detections": [], "frameSize": {"width": width, "height": height}}
        undistorted = cv2.undistort(image, matrix, distortion)
        fiducials = calibration.get("fiducials") or {}
        dictionary_name = str(fiducials.get("dictionary") or DICTIONARY_NAME)
        corners, ids, rejected = self._detector(dictionary_name).detectMarkers(undistorted)
        ids_flat = [] if ids is None else [int(value) for value in ids.flatten()]
        reference = {int(item["id"]): item for item in fiducials.get("referenceMarkers") or [] if "id" in item}
        definitions = []
        for item in (registered_parts if registered_parts is not None else fiducials.get("objectTags") or []):
            definitions.append({**item, "entityKind": "part", "entityId": item.get("partId")})
        for item in registered_bins or []:
            definitions.append({**item, "entityKind": "bin", "entityId": item.get("binId")})
        object_tags = {int(item.get("tagId", item.get("id"))): item for item in definitions if item.get("tagId", item.get("id")) is not None}
        object_tag_ids = set(range(10, 26))
        unknown_ids = sorted(set(ids_flat) - set(reference) - object_tag_ids)
        visible_tags = []
        for marker_corners, marker_id in zip(corners, ids_flat):
            if marker_id in object_tag_ids:
                points = marker_corners.reshape(4, 2)
                definition = object_tags.get(marker_id)
                visible_tags.append({
                    "tagId": marker_id, "cornersPx": points.tolist(), "centerPx": points.mean(axis=0).tolist(),
                    "bound": bool(definition), "partId": (definition or {}).get("partId"),
                    "binId": (definition or {}).get("binId"),
                    "bindingKind": (definition or {}).get("entityKind"),
                    "entityId": (definition or {}).get("entityId"),
                    "label": (definition or {}).get("label"),
                })

        # Object-tag selection is an image-space operation.  Keep those
        # candidates available even when robot-coordinate localization is
        # rejected (most importantly when the camera pose must be re-locked).
        # Invalid frames still contain no detections or robot coordinates.
        def reject(reason: str, quality: Dict[str, Any], homography_value=None) -> Dict[str, Any]:
            for tag in visible_tags:
                tag.update({
                    "localizationStatus": "frame_invalid" if tag.get("bound") else "unbound",
                    "rejectionReason": reason if tag.get("bound") else None,
                })
            quality["objectTagStatuses"] = [
                {
                    "tagId": tag["tagId"], "partId": tag.get("partId"), "binId": tag.get("binId"),
                    "bindingKind": tag.get("bindingKind"), "entityId": tag.get("entityId"),
                    "localizationStatus": tag["localizationStatus"],
                    "reason": tag.get("rejectionReason"),
                }
                for tag in visible_tags if tag.get("bound")
            ]
            result = self._invalid(reason, undistorted, quality, corners, ids_flat, draw_debug)
            result["frameSize"] = {"width": width, "height": height}
            result["visibleTags"] = visible_tags
            if homography_value is not None:
                result["homography"] = homography_value
            return result

        pixel_points: List[List[float]] = []
        robot_points: List[List[float]] = []
        point_marker_ids: List[int] = []
        visible_reference_ids = []
        marker_size = float(fiducials.get("markerSizeM") or DEFAULT_MARKER_SIZE_M)
        layout_errors = reference_layout_errors(reference, marker_size)
        if layout_errors:
            quality = {"visibleMarkerCount": 0, "visibleMarkerIds": [], "unknownMarkerIds": unknown_ids, "referenceLayoutErrors": layout_errors}
            return reject("reference_layout_invalid", quality)
        for marker_corners, marker_id in zip(corners, ids_flat):
            if marker_id in reference:
                visible_reference_ids.append(marker_id)
                pixel_points.extend(marker_corners.reshape(4, 2).tolist())
                robot_points.extend(marker_robot_corners(reference[marker_id], marker_size).tolist())
                point_marker_ids.extend([marker_id] * 4)
        quality: Dict[str, Any] = {"visibleMarkerCount": len(set(visible_reference_ids)), "visibleMarkerIds": sorted(set(visible_reference_ids)), "unknownMarkerIds": unknown_ids}
        if unknown_ids:
            return reject("unknown_marker_ids", quality)
        if quality["visibleMarkerCount"] < int(fiducials.get("minimumMarkers") or MIN_MARKERS):
            return reject("insufficient_reference_markers", quality)
        missing_reference_ids = sorted(set(reference) - set(visible_reference_ids))
        if missing_reference_ids:
            quality["missingReferenceIds"] = missing_reference_ids
            return reject("not_all_reference_markers_visible", quality)
        src = np.asarray(pixel_points, np.float64)
        dst = np.asarray(robot_points, np.float64)
        homography, mask = cv2.findHomography(src, dst, cv2.RANSAC, 0.004)
        if homography is None:
            return reject("homography_failed", quality)
        normalized = homography / homography[2, 2]
        condition = float(np.linalg.cond(normalized))
        inverse = np.linalg.inv(homography)
        projected_px = cv2.perspectiveTransform(dst.reshape(-1, 1, 2), inverse).reshape(-1, 2)
        errors = np.linalg.norm(projected_px - src, axis=1)
        inliers = np.ones(len(errors), dtype=bool) if mask is None else mask.reshape(-1).astype(bool)
        inlier_errors = errors[inliers]
        all_rms = float(math.sqrt(float(np.mean(errors * errors))))
        all_maximum = float(np.max(errors))
        inlier_rms = float(math.sqrt(float(np.mean(inlier_errors * inlier_errors)))) if len(inlier_errors) else float("inf")
        inlier_maximum = float(np.max(inlier_errors)) if len(inlier_errors) else float("inf")
        per_marker = []
        failing_marker_ids = []
        insufficient_inlier_ids = []
        labels = np.asarray(point_marker_ids)
        for marker_id in sorted(set(point_marker_ids)):
            selected = labels == marker_id
            marker_errors = errors[selected]
            marker_inliers = int(np.sum(inliers[selected]))
            marker_rms = float(math.sqrt(float(np.mean(marker_errors * marker_errors))))
            marker_maximum = float(np.max(marker_errors))
            passed = marker_inliers >= 2 and marker_rms <= float(fiducials.get("maxReprojectionRmsPx") or MAX_REPROJECTION_RMS_PX) and marker_maximum <= float(fiducials.get("maxReprojectionPx") or MAX_REPROJECTION_PX)
            if marker_inliers < 2:
                insufficient_inlier_ids.append(marker_id)
            if not passed:
                failing_marker_ids.append(marker_id)
            per_marker.append({
                "id": marker_id, "rmsPx": marker_rms, "maxPx": marker_maximum,
                "inlierCornerCount": marker_inliers, "cornerCount": int(np.sum(selected)), "passed": passed,
            })
        hull = cv2.convexHull(src.astype(np.float32))
        coverage = float(cv2.contourArea(hull) / max(1.0, width * height))
        quality.update({
            "conditionNumber": condition, "reprojectionRmsPx": all_rms, "reprojectionMaxPx": all_maximum,
            "allCornerRmsPx": all_rms, "allCornerMaxPx": all_maximum,
            "inlierRmsPx": inlier_rms, "inlierMaxPx": inlier_maximum,
            "inlierCornerCount": int(np.sum(inliers)), "cornerCount": len(errors),
            "inlierRatio": float(np.mean(inliers)), "perMarker": per_marker,
            "failingMarkerIds": failing_marker_ids, "coverageRatio": coverage,
        })
        if condition > float(fiducials.get("maxConditionNumber") or MAX_CONDITION_NUMBER):
            return reject("homography_poorly_conditioned", quality)
        if insufficient_inlier_ids:
            return reject("marker_inliers_insufficient", quality)
        if all_rms > float(fiducials.get("maxReprojectionRmsPx") or MAX_REPROJECTION_RMS_PX) or all_maximum > float(fiducials.get("maxReprojectionPx") or MAX_REPROJECTION_PX):
            return reject("reprojection_error_excessive", quality)
        if coverage < float(fiducials.get("minimumCoverageRatio") or MIN_COVERAGE_RATIO):
            return reject("marker_coverage_insufficient", quality)
        baseline = fiducials.get("baselineHomography")
        if baseline:
            baseline_h = np.asarray(baseline, np.float64)
            probes = np.asarray([[[0, 0]], [[width, 0]], [[width, height]], [[0, height]], [[width / 2, height / 2]]], np.float64)
            current_xy = cv2.perspectiveTransform(probes, homography).reshape(-1, 2)
            baseline_xy = cv2.perspectiveTransform(probes, baseline_h).reshape(-1, 2)
            drift = float(np.max(np.linalg.norm(current_xy - baseline_xy, axis=1)))
            quality["cameraDriftM"] = drift
            if drift > float(fiducials.get("cameraMoveLimitM") or CAMERA_MOVE_LIMIT_M) and not fiducials.get("allowCurrentPose", False):
                return reject("camera_moved_reaccept_required", quality, normalized.tolist())
        object_points_3d = np.column_stack((dst, np.zeros(len(dst), np.float64)))
        pose_ok, rvec, tvec = cv2.solvePnP(object_points_3d, src, matrix, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
        if not pose_ok:
            return reject("camera_pose_failed", quality)
        detections = self._tagged_detections(
            corners, ids_flat, object_tags, matrix, rvec, tvec, quality,
            support_surfaces=support_surfaces,
        )
        quality["objectTagRejections"] = deepcopy(self.last_object_tag_rejections)
        quality["objectTagStatuses"] = deepcopy(list(self.last_object_tag_statuses.values()))
        detections = self.stabilizer.update(detections)
        detection_by_tag = {int(item.get("tagId")): item for item in detections if item.get("tagId") is not None}
        for tag in visible_tags:
            x, y = apply_homography(normalized, tag["centerPx"])
            tag["robotTablePosition"] = {"x": x, "y": y, "z": 0.0}
            status = deepcopy(self.last_object_tag_statuses.get(int(tag["tagId"])) or {})
            if not tag.get("bound"):
                status = {"localizationStatus": "unbound", "rejectionReason": None}
            elif int(tag["tagId"]) in detection_by_tag:
                status.update({"localizationStatus": "localized", "rejectionReason": None})
            if status.get("reason") and "rejectionReason" not in status:
                status["rejectionReason"] = status["reason"]
            tag.update(status)
        overlay = self._draw_overlay(undistorted, corners, ids_flat, detections, quality, None) if draw_debug else None
        return {
            "ok": True, "frame": "robot", "detections": detections, "quality": quality,
            "homography": normalized.tolist(), "frameSize": {"width": width, "height": height},
            "visibleTags": visible_tags,
            "debugJpeg": encode_jpeg(overlay) if overlay is not None else None,
        }

    @staticmethod
    def _pixels_on_robot_plane(points, matrix, rvec, tvec, plane_z):
        rotation, _ = cv2.Rodrigues(rvec)
        camera_center = (-rotation.T @ np.asarray(tvec, np.float64).reshape(3, 1)).reshape(3)
        inverse_matrix = np.linalg.inv(matrix)
        output = []
        for px, py in np.asarray(points, np.float64).reshape(-1, 2):
            ray_camera = inverse_matrix @ np.asarray([px, py, 1.0], np.float64)
            ray_robot = rotation.T @ ray_camera
            if abs(float(ray_robot[2])) < 1e-9:
                raise ValueError("camera ray is parallel to object top plane")
            scale = (float(plane_z) - float(camera_center[2])) / float(ray_robot[2])
            output.append(camera_center + scale * ray_robot)
        return np.asarray(output, np.float64)

    @staticmethod
    def _surface_contains(surface: Dict[str, Any], x: float, y: float) -> bool:
        center = surface.get("center") or {}
        size = surface.get("size") or {}
        return (
            abs(float(x) - float(center.get("x", 0.0))) <= float(size.get("x", 0.0)) / 2.0
            and abs(float(y) - float(center.get("y", 0.0))) <= float(size.get("y", 0.0)) / 2.0
        )

    @staticmethod
    def _surface_edge_gaps(surface: Dict[str, Any], x: float, y: float) -> Tuple[float, float]:
        center = surface.get("center") or {}
        size = surface.get("size") or {}
        return (
            max(0.0, abs(float(x) - float(center.get("x", 0.0))) - float(size.get("x", 0.0)) / 2.0),
            max(0.0, abs(float(y) - float(center.get("y", 0.0))) - float(size.get("y", 0.0)) / 2.0),
        )

    @staticmethod
    def _tag_pose_candidates(points, expected_size, matrix, robot_rvec, robot_tvec):
        half = float(expected_size) / 2.0
        tag_points = np.asarray([
            [half, half, 0.0], [half, -half, 0.0],
            [-half, -half, 0.0], [-half, half, 0.0],
        ], np.float64)
        pose_ok, tag_rvec, tag_tvec = cv2.solvePnP(
            tag_points, np.asarray(points, np.float64).reshape(4, 2), matrix,
            np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not pose_ok:
            return []
        camera_rotation, _ = cv2.Rodrigues(robot_rvec)
        robot_translation = np.asarray(robot_tvec, np.float64).reshape(3)
        candidates = []
        for tag_rvec, tag_tvec in ((tag_rvec, tag_tvec),):
            tag_rotation, _ = cv2.Rodrigues(tag_rvec)
            tag_translation = np.asarray(tag_tvec, np.float64).reshape(3)
            center_robot = camera_rotation.T @ (tag_translation - robot_translation)
            rotation_robot = camera_rotation.T @ tag_rotation
            projected, _ = cv2.projectPoints(tag_points, tag_rvec, tag_tvec, matrix, np.zeros(5))
            errors = np.linalg.norm(projected.reshape(4, 2) - np.asarray(points).reshape(4, 2), axis=1)
            normal_z = float(rotation_robot[2, 2])
            candidates.append({
                "center": center_robot,
                "rotation": rotation_robot,
                "rmsPx": float(math.sqrt(float(np.mean(errors * errors)))),
                "maxPx": float(np.max(errors)),
                "normalZ": normal_z,
            })
        return sorted(
            candidates,
            key=lambda item: (
                item["rmsPx"]
                + (100.0 if item["center"][2] < -0.02 or item["center"][2] > 0.40 else 0.0)
                + (50.0 if abs(item["normalZ"]) < math.cos(math.radians(15.0)) else 0.0)
            ),
        )

    @classmethod
    def _horizontal_tag_pose(cls, points, expected_size, matrix, robot_rvec, robot_tvec,
                             minimum_z: float = -0.02, maximum_z: float = 0.40):
        """Recover a flat tag's height from its known size without planar PnP tilt ambiguity."""
        points = np.asarray(points, np.float64).reshape(4, 2)
        expected_size = float(expected_size)

        def geometry_at(z_value: float):
            corners = cls._pixels_on_robot_plane(points, matrix, robot_rvec, robot_tvec, z_value)
            xy = corners[:, :2]
            edges = np.roll(xy, -1, axis=0) - xy
            sides = np.linalg.norm(edges, axis=1)
            diagonals = np.asarray([
                np.linalg.norm(xy[2] - xy[0]), np.linalg.norm(xy[3] - xy[1]),
            ])
            adjacent_cosines = []
            for index in range(4):
                a = edges[index]
                b = edges[(index + 1) % 4]
                adjacent_cosines.append(abs(float(np.dot(a, b))) / max(1e-12, float(np.linalg.norm(a) * np.linalg.norm(b))))
            mean_side = float(np.mean(sides))
            quality = {
                "meanSideM": mean_side,
                "sideVariationRatio": float(np.std(sides) / max(mean_side, 1e-9)),
                "diagonalMismatchRatio": float(abs(diagonals[0] - diagonals[1]) / max(float(np.mean(diagonals)), 1e-9)),
                "maximumAdjacentCosine": float(max(adjacent_cosines)),
            }
            return corners, quality

        camera_rotation, _ = cv2.Rodrigues(robot_rvec)
        camera_center = (-camera_rotation.T @ np.asarray(robot_tvec, np.float64).reshape(3, 1)).reshape(3)
        upper = min(float(maximum_z), float(camera_center[2]) - 0.02)
        lower = float(minimum_z)
        if upper <= lower:
            return None, "object_tag_height_unavailable"
        try:
            _, low_quality = geometry_at(lower)
            _, high_quality = geometry_at(upper)
        except (ValueError, np.linalg.LinAlgError):
            return None, "object_tag_height_unavailable"
        low_error = low_quality["meanSideM"] - expected_size
        high_error = high_quality["meanSideM"] - expected_size
        if low_error * high_error > 0.0:
            return None, "object_tag_scale_inconsistent"
        for _ in range(42):
            middle = (lower + upper) / 2.0
            _, middle_quality = geometry_at(middle)
            middle_error = middle_quality["meanSideM"] - expected_size
            if low_error * middle_error <= 0.0:
                upper, high_error = middle, middle_error
            else:
                lower, low_error = middle, middle_error
        measured_z = (lower + upper) / 2.0
        corners, geometry = geometry_at(measured_z)
        geometry["measuredTagSizeM"] = geometry["meanSideM"]
        geometry["expectedTagSizeM"] = expected_size
        geometry_ok = (
            geometry["sideVariationRatio"] <= 0.18
            and geometry["diagonalMismatchRatio"] <= 0.15
            and geometry["maximumAdjacentCosine"] <= 0.30
        )
        if not geometry_ok:
            return {"z": measured_z, "corners": corners, "geometry": geometry}, "object_tag_geometry_inconsistent"
        center = corners[:, :2].mean(axis=0)
        edge = corners[1, :2] - corners[0, :2]
        yaw = math.degrees(math.atan2(float(edge[1]), float(edge[0]))) + 90.0
        return {"z": measured_z, "corners": corners, "center": center, "yawDeg": yaw, "geometry": geometry}, None

    def _committed_surface(
        self, part_id: str, measured_support_z: float, x: float, y: float,
        surfaces: List[Dict[str, Any]], now: float,
        surface_positions: Optional[Dict[str, Tuple[float, float]]] = None,
        footprint_margin: Tuple[float, float] = (0.0, 0.0),
    ) -> Tuple[Optional[Dict[str, Any]], str, float]:
        # A 30 mm tag near the edge of a camera frame can move the recovered
        # support height by a couple of millimetres even when its square
        # geometry is otherwise valid.  Keep the operator's configured entry
        # tolerance as the nominal limit, but add a very small measurement
        # guard band at that boundary.  This is deliberately much smaller
        # than the separation required to choose between two surfaces, so it
        # cannot turn an ambiguous Table/Platform observation into a match.
        entry_measurement_guard_m = 0.0025
        track = self.surface_tracks.setdefault(part_id, {
            "heights": deque(maxlen=5), "candidateId": None, "candidateCount": 0,
            "surfaceId": None, "updatedAt": now,
        })
        if now - float(track.get("updatedAt") or 0.0) > 1.0:
            track["heights"].clear()
            track["candidateId"] = None
            track["candidateCount"] = 0
        track["heights"].append(float(measured_support_z))
        track["updatedAt"] = now
        filtered_z = float(statistics.median(track["heights"]))
        enabled = []
        footprint_rejections = []
        overlap_by_surface = {}
        margin_x = min(0.025, max(0.0, float(footprint_margin[0])))
        margin_y = min(0.025, max(0.0, float(footprint_margin[1])))
        for item in surfaces:
            if not item.get("enabled", True):
                continue
            candidate_x, candidate_y = (surface_positions or {}).get(
                str(item.get("id")), (float(x), float(y))
            )
            raw_dx, raw_dy = self._surface_edge_gaps(item, candidate_x, candidate_y)
            if raw_dx <= margin_x and raw_dy <= margin_y:
                enabled.append(item)
                overlap_by_surface[str(item.get("id"))] = {
                    "edgeOverhangM": max(raw_dx, raw_dy),
                    "usedObjectFootprintOverlap": bool(raw_dx > 0.0 or raw_dy > 0.0),
                }
            else:
                footprint_rejections.append({
                    "surfaceId": item.get("id"),
                    "distanceM": math.hypot(max(0.0, raw_dx - margin_x), max(0.0, raw_dy - margin_y)),
                    "rawEdgeGapM": max(raw_dx, raw_dy),
                    "heightResidualM": abs(filtered_z - float(item.get("topZ") or 0.0)),
                    "candidatePosition": {"x": candidate_x, "y": candidate_y},
                })
        matches = []
        for surface in enabled:
            tolerance_key = "holdToleranceM" if surface.get("id") == track.get("surfaceId") else "entryToleranceM"
            tolerance = float(surface.get(tolerance_key) or (0.020 if tolerance_key == "holdToleranceM" else 0.015))
            effective_tolerance = tolerance + (entry_measurement_guard_m if tolerance_key == "entryToleranceM" else 0.0)
            residual = abs(filtered_z - float(surface.get("topZ") or 0.0))
            if residual <= effective_tolerance:
                matches.append((residual, surface))
        matches.sort(key=lambda item: item[0])
        if len(matches) > 1 and matches[1][0] - matches[0][0] >= 0.010:
            matches = matches[:1]
        track["surfaceDiagnostics"] = {
            "candidateCount": len(matches),
            "nearestSurfaceId": matches[0][1].get("id") if matches else (min(enabled, key=lambda item: abs(filtered_z - float(item.get("topZ") or 0.0))).get("id") if enabled else None),
            "nearestResidualM": matches[0][0] if matches else (min((abs(filtered_z - float(item.get("topZ") or 0.0)) for item in enabled), default=None)),
            "stabilizationCount": int(track.get("candidateCount") or 0),
            "stabilizationRequired": 3,
            "footprintRejections": sorted(footprint_rejections, key=lambda item: (item["heightResidualM"], item["distanceM"])),
            "surfaceOverlap": overlap_by_surface,
            "entryMeasurementGuardM": entry_measurement_guard_m,
        }
        if len(matches) != 1:
            track["candidateId"] = None
            track["candidateCount"] = 0
            close_footprint = next((
                item for item in track["surfaceDiagnostics"]["footprintRejections"]
                if item["heightResidualM"] <= 0.020
            ), None)
            if close_footprint:
                track["surfaceDiagnostics"]["nearestSurfaceId"] = close_footprint["surfaceId"]
                track["surfaceDiagnostics"]["nearestResidualM"] = close_footprint["heightResidualM"]
                track["surfaceDiagnostics"]["footprintDistanceM"] = close_footprint["distanceM"]
            reason = (
                "support_surface_ambiguous" if len(matches) > 1
                else "support_surface_outside_footprint" if close_footprint
                else "support_surface_unknown"
            )
            return None, reason, filtered_z
        matches.sort(key=lambda item: item[0])
        selected = matches[0][1]
        track["surfaceDiagnostics"].update(overlap_by_surface.get(str(selected.get("id"))) or {})
        if track.get("surfaceId") == selected.get("id"):
            return selected, "matched", filtered_z
        if track.get("candidateId") == selected.get("id"):
            track["candidateCount"] += 1
        else:
            track["candidateId"] = selected.get("id")
            track["candidateCount"] = 1
        if track["candidateCount"] < 3:
            track["surfaceDiagnostics"]["stabilizationCount"] = track["candidateCount"]
            return None, "support_surface_stabilizing", filtered_z
        track["surfaceId"] = selected.get("id")
        track["candidateId"] = None
        track["candidateCount"] = 0
        return selected, "matched", filtered_z

    def _tagged_detections(
        self, corners, ids, mappings, matrix, rvec, tvec, quality,
        support_surfaces: Optional[List[Dict[str, Any]]] = None,
    ):
        out = []
        self.last_object_tag_rejections = []
        self.last_object_tag_statuses = {}
        now = time.time()
        for marker_corners, marker_id in zip(corners, ids):
            config = mappings.get(marker_id)
            if not config:
                continue
            points = marker_corners.reshape(4, 2)
            entity_kind = str(config.get("entityKind") or ("bin" if config.get("binId") else "part"))
            entity_id = str(config.get("entityId") or config.get("binId") or config.get("partId") or config.get("id") or "")
            size = {**{"x": 0.04, "y": 0.04, "z": 0.05}, **(config.get("outer") or config.get("size") or {})}
            mount_height = float(config.get("mountHeightM", size["z"]) if entity_kind == "bin" else size["z"])
            expected_size = float(config.get("tagSizeM") or 0.03)
            surface_diagnostics = {}
            # Backward-compatible direct-call path for old fixtures and saved
            # calibrations. The continuous runtime always supplies surfaces
            # and therefore uses the true size-derived 3D tag pose below.
            if support_surfaces is None:
                robot_corners = self._pixels_on_robot_plane(points, matrix, rvec, tvec, float(size["z"]))
                side_lengths = np.linalg.norm(np.roll(robot_corners[:, :2], -1, axis=0) - robot_corners[:, :2], axis=1)
                recovered_size = float(np.mean(side_lengths))
                if not 0.65 * expected_size <= recovered_size <= 1.35 * expected_size:
                    continue
                tag_center = robot_corners[:, :2].mean(axis=0)
                tag_yaw = math.degrees(math.atan2(robot_corners[1, 1] - robot_corners[0, 1], robot_corners[1, 0] - robot_corners[0, 0])) + 90.0
                surface = {"id": "surface-table", "name": "Main Table", "topZ": 0.0}
                measured_tag_z = float(size["z"])
                measured_support_z = 0.0
                pose_rms = 0.0
                pose_maximum = 0.0
                tilt_deg = 0.0
                geometry_quality = None
            else:
                maximum_z = max(
                    [float(item.get("topZ") or 0.0) + mount_height + 0.08 for item in support_surfaces]
                    or [0.40]
                )
                pose, pose_error = self._horizontal_tag_pose(
                    points, expected_size, matrix, rvec, tvec,
                    minimum_z=-0.02, maximum_z=maximum_z,
                )
                geometry_quality = deepcopy((pose or {}).get("geometry"))
                if pose_error:
                    rejection = {
                        "tagId": marker_id, "partId": config.get("partId"), "binId": config.get("binId"),
                        "bindingKind": entity_kind, "entityId": entity_id,
                        "reason": pose_error, "localizationStatus": "rejected",
                        "tagGeometryQuality": geometry_quality,
                    }
                    self.last_object_tag_rejections.append(rejection)
                    self.last_object_tag_statuses[marker_id] = deepcopy(rejection)
                    continue
                tag_center = pose["center"]
                measured_tag_z = float(pose["z"])
                measured_support_z = measured_tag_z - mount_height
                track_key = f"{entity_kind}:{entity_id or marker_id}"
                surface_positions = {}
                footprint_margin = (0.0, 0.0)
                tag_offset = config.get("tagOffsetM") or config.get("centerOffsetM") or {}
                for candidate_surface in support_surfaces:
                    candidate_z = float(candidate_surface.get("topZ") or 0.0) + mount_height
                    candidate_corners = self._pixels_on_robot_plane(points, matrix, rvec, tvec, candidate_z)
                    candidate_center = candidate_corners[:, :2].mean(axis=0)
                    candidate_edge = candidate_corners[1, :2] - candidate_corners[0, :2]
                    candidate_tag_yaw = math.degrees(math.atan2(float(candidate_edge[1]), float(candidate_edge[0]))) + 90.0
                    candidate_yaw = candidate_tag_yaw + float(config.get("yawOffsetDeg") or 0.0)
                    candidate_radians = math.radians(candidate_yaw)
                    candidate_ox = float(tag_offset.get("x", 0.0))
                    candidate_oy = float(tag_offset.get("y", 0.0))
                    candidate_object_x = float(candidate_center[0]) - math.cos(candidate_radians) * candidate_ox + math.sin(candidate_radians) * candidate_oy
                    candidate_object_y = float(candidate_center[1]) - math.sin(candidate_radians) * candidate_ox - math.cos(candidate_radians) * candidate_oy
                    surface_positions[str(candidate_surface.get("id"))] = (
                        candidate_object_x, candidate_object_y,
                    )
                    footprint_margin = (
                        abs(math.cos(candidate_radians)) * float(size["x"]) / 2.0
                        + abs(math.sin(candidate_radians)) * float(size["y"]) / 2.0,
                        abs(math.sin(candidate_radians)) * float(size["x"]) / 2.0
                        + abs(math.cos(candidate_radians)) * float(size["y"]) / 2.0,
                    )
                surface, surface_reason, filtered_support_z = self._committed_surface(
                    track_key,
                    measured_support_z, float(tag_center[0]), float(tag_center[1]),
                    list(support_surfaces), now, surface_positions=surface_positions,
                    footprint_margin=footprint_margin,
                )
                surface_diagnostics = deepcopy(
                    (self.surface_tracks.get(track_key) or {}).get("surfaceDiagnostics") or {}
                )
                if surface is None:
                    rejection = {
                        "tagId": marker_id, "partId": config.get("partId"), "binId": config.get("binId"),
                        "bindingKind": entity_kind, "entityId": entity_id, "reason": surface_reason,
                        "localizationStatus": "stabilizing" if surface_reason == "support_surface_stabilizing" else "rejected",
                        "measuredTagTopZ": measured_tag_z, "measuredSupportZ": filtered_support_z,
                        "nearestSurfaceId": surface_diagnostics.get("nearestSurfaceId"),
                        "nearestSurfaceResidualM": surface_diagnostics.get("nearestResidualM"),
                        "surfaceFootprintDistanceM": surface_diagnostics.get("footprintDistanceM"),
                        "edgeOverhangM": surface_diagnostics.get("edgeOverhangM"),
                        "measuredTagPosition": {"x": float(tag_center[0]), "y": float(tag_center[1])},
                        "tagGeometryQuality": geometry_quality,
                        "stabilizationProgress": {
                            "frames": surface_diagnostics.get("stabilizationCount", 0),
                            "required": surface_diagnostics.get("stabilizationRequired", 3),
                        },
                    }
                    self.last_object_tag_rejections.append(rejection)
                    self.last_object_tag_statuses[marker_id] = deepcopy(rejection)
                    continue
                measured_support_z = filtered_support_z
                tag_yaw = float(pose["yawDeg"])
                pose_rms = 0.0
                pose_maximum = 0.0
                tilt_deg = None
                canonical_tag_z = float(surface.get("topZ") or 0.0) + mount_height
                canonical_corners = self._pixels_on_robot_plane(points, matrix, rvec, tvec, canonical_tag_z)
                side_lengths = np.linalg.norm(np.roll(canonical_corners[:, :2], -1, axis=0) - canonical_corners[:, :2], axis=1)
                recovered_size = float(np.mean(side_lengths))
                if not 0.65 * expected_size <= recovered_size <= 1.35 * expected_size:
                    rejection = {
                        "tagId": marker_id, "partId": config.get("partId"), "binId": config.get("binId"),
                        "bindingKind": entity_kind, "entityId": entity_id,
                        "reason": "object_tag_size_inconsistent", "localizationStatus": "rejected",
                        "measuredTagTopZ": measured_tag_z, "measuredSupportZ": measured_support_z,
                        "nearestSurfaceId": surface.get("id"),
                        "nearestSurfaceResidualM": abs(measured_support_z - float(surface.get("topZ") or 0.0)),
                        "tagGeometryQuality": geometry_quality,
                    }
                    self.last_object_tag_rejections.append(rejection)
                    self.last_object_tag_statuses[marker_id] = deepcopy(rejection)
                    continue
                tag_center = canonical_corners[:, :2].mean(axis=0)
            yaw = tag_yaw + float(config.get("yawOffsetDeg") or 0.0)
            offset = config.get("tagOffsetM") or config.get("centerOffsetM") or {}
            radians = math.radians(yaw)
            ox, oy = float(offset.get("x", 0.0)), float(offset.get("y", 0.0))
            x = float(tag_center[0]) - math.cos(radians) * ox + math.sin(radians) * oy
            y = float(tag_center[1]) - math.sin(radians) * ox - math.cos(radians) * oy
            source = "bin_tag" if entity_kind == "bin" else "object_tag"
            detection = self._detection(entity_id or f"tag-object-{marker_id}", config.get("label") or f"Tagged {entity_kind.title()} {marker_id}", config.get("type") or config.get("class") or "box", x, y, yaw, size, points, source, quality, 0.99)
            detection["position"]["z"] = float(surface.get("topZ") or 0.0) if entity_kind == "bin" else float(surface.get("topZ") or 0.0) + float(size["z"]) / 2.0
            detection.update({
                "tagId": marker_id,
                "entityKind": entity_kind, "entityId": entity_id,
                "partId": config.get("partId"), "binId": config.get("binId"),
                "supportSurfaceId": surface.get("id"),
                "supportSurfaceName": surface.get("name"),
                "supportSurfaceZ": float(surface.get("topZ") or 0.0),
                "measuredTagTopZ": measured_tag_z,
                "measuredSupportZ": measured_support_z,
                "supportHeightResidualM": measured_support_z - float(surface.get("topZ") or 0.0),
                "poseQuality": {
                    "recoveredTagSizeM": recovered_size,
                    "expectedTagSizeM": expected_size,
                    "reprojectionRmsPx": pose_rms,
                    "reprojectionMaxPx": pose_maximum,
                    "tagTiltDeg": tilt_deg,
                    "tagGeometryQuality": geometry_quality,
                    "edgeOverhangM": surface_diagnostics.get("edgeOverhangM", 0.0),
                    "usedObjectFootprintOverlap": bool(surface_diagnostics.get("usedObjectFootprintOverlap")),
                    "surfaceConfidence": max(0.0, 1.0 - abs(measured_support_z - float(surface.get("topZ") or 0.0)) / max(0.001, float(surface.get("holdToleranceM") or 0.020))),
                },
            })
            out.append(detection)
            self.last_object_tag_statuses[marker_id] = {
                "tagId": marker_id, "partId": config.get("partId"), "binId": config.get("binId"),
                "bindingKind": entity_kind, "entityId": entity_id,
                "localizationStatus": "localized", "rejectionReason": None,
                "measuredTagTopZ": measured_tag_z, "measuredSupportZ": measured_support_z,
                "nearestSurfaceId": surface.get("id"),
                "nearestSurfaceResidualM": abs(measured_support_z - float(surface.get("topZ") or 0.0)),
                "tagGeometryQuality": geometry_quality,
                "edgeOverhangM": surface_diagnostics.get("edgeOverhangM", 0.0),
                "usedObjectFootprintOverlap": bool(surface_diagnostics.get("usedObjectFootprintOverlap")),
                "stabilizationProgress": {"frames": 3, "required": 3},
            }
        return out

    @staticmethod
    def _detection(identifier, label, cls, x, y, yaw, size, points, source, quality, confidence):
        points = np.asarray(points).reshape(-1, 2)
        bx, by, bw, bh = cv2.boundingRect(points.astype(np.float32))
        size = {k: float(size.get(k, 0.035)) for k in ("x", "y", "z")}
        return {
            "id": str(identifier), "trackId": str(identifier), "label": str(label), "class": str(cls), "type": "box",
            "confidence": float(confidence), "bboxPx": {"x": bx, "y": by, "width": bw, "height": bh},
            "position": {"x": float(x), "y": float(y), "z": size["z"] / 2}, "size": size,
            "orientationDeg": ((float(yaw) + 180) % 360) - 180, "localizationSource": source,
            "calibrationQuality": {k: quality.get(k) for k in ("visibleMarkerCount", "reprojectionRmsPx", "reprojectionMaxPx", "coverageRatio")},
            "timestamp": time.time(),
        }

    def _invalid(self, reason, image, quality, corners, ids, draw):
        overlay = self._draw_overlay(image, corners, ids, [], quality, reason) if draw else None
        return {"ok": False, "error": reason, "detections": [], "quality": quality, "debugJpeg": encode_jpeg(overlay) if overlay is not None else None}

    @staticmethod
    def _draw_overlay(image, corners, ids, detections, quality, error):
        overlay = image.copy()
        if ids:
            cv2.aruco.drawDetectedMarkers(overlay, corners, np.asarray(ids, np.int32).reshape(-1, 1))
        object_statuses = {
            int(item.get("tagId")): item for item in (quality.get("objectTagStatuses") or [])
            if item.get("tagId") is not None
        }
        for marker_corners, marker_id in zip(corners, ids or []):
            status = object_statuses.get(int(marker_id))
            if not status:
                continue
            state = status.get("localizationStatus")
            color = (20, 210, 40) if state == "localized" else ((0, 180, 255) if state == "stabilizing" else (20, 20, 230))
            polygon = np.asarray(marker_corners, np.int32).reshape(-1, 1, 2)
            cv2.polylines(overlay, [polygon], True, color, 3, cv2.LINE_AA)
            anchor = tuple(np.asarray(marker_corners).reshape(-1, 2).min(axis=0).astype(int))
            label = f"Tag {marker_id}: {state}"
            if status.get("reason"):
                label += f" ({status['reason']})"
            cv2.putText(overlay, label, (anchor[0], max(18, anchor[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
        for det in detections:
            box = det["bboxPx"]
            p1, p2 = (int(box["x"]), int(box["y"])), (int(box["x"] + box["width"]), int(box["y"] + box["height"]))
            cv2.rectangle(overlay, p1, p2, (30, 220, 60), 2)
            text = f"{det.get('trackId')} {det['position']['x']:.3f},{det['position']['y']:.3f}"
            cv2.putText(overlay, text, (p1[0], max(18, p1[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 220, 60), 1, cv2.LINE_AA)
        status = error or f"markers={quality.get('visibleMarkerCount', 0)} rms={quality.get('reprojectionRmsPx', 0):.2f}px"
        cv2.putText(overlay, status, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255) if error else (0, 220, 0), 2, cv2.LINE_AA)
        if quality.get("conditionNumber") is not None:
            details = f"cond={quality['conditionNumber']:.0f} inliers={quality.get('inlierCornerCount', 0)}/{quality.get('cornerCount', 0)}"
            cv2.putText(overlay, details, (12, 49), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 255) if error else (0, 180, 0), 1, cv2.LINE_AA)
        y = 73
        for marker in quality.get("perMarker") or []:
            color = (0, 180, 0) if marker.get("passed") else (0, 0, 255)
            label = f"id {marker['id']}: rms {marker['rmsPx']:.1f}px max {marker['maxPx']:.1f}px in {marker['inlierCornerCount']}/4"
            cv2.putText(overlay, label, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)
            y += 20
        return overlay


def verification_report(samples: List[Dict[str, Any]], stationary_spread_m: Optional[float] = None) -> Dict[str, Any]:
    errors = [math.hypot(float(s["measured"]["x"]) - float(s["expected"]["x"]), float(s["measured"]["y"]) - float(s["expected"]["y"])) for s in samples]
    # JSON has no representation for Infinity. Missing measurements are an
    # incomplete report, not an infinitely inaccurate calibration.
    rms = math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None
    maximum = max(errors) if errors else None
    spread = None if stationary_spread_m is None else float(stationary_spread_m)
    return {
        "sampleCount": len(errors), "rmsXyErrorM": rms, "maxXyErrorM": maximum,
        "stationarySpreadM": spread,
        "passed": len(errors) >= 9 and rms <= MAX_VERIFICATION_RMS_M and maximum <= MAX_VERIFICATION_ERROR_M and spread is not None and spread <= MAX_STATIONARY_SPREAD_M,
    }


class ContinuousLocalizationRuntime:
    """Background bridge between CameraService frames and Workcell detections."""

    def __init__(self, camera: Any, scene: Any) -> None:
        self.camera = camera
        self.scene = scene
        self.localizer = FiducialLocalizer()
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.process_lock = threading.Lock()
        self.last_result: Dict[str, Any] = {"ok": False, "error": "not_started", "detections": []}
        self.last_debug_jpeg: Optional[bytes] = None
        self.last_visible_tags: List[Dict[str, Any]] = []
        self.processed_frames = 0

    def start(self) -> Dict[str, Any]:
        if self.thread and self.thread.is_alive():
            return self.status()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._loop, name="fiducial-localization", daemon=True)
        self.thread.start()
        return self.status()

    def stop(self) -> Dict[str, Any]:
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.thread = None
        self.scene.ingest_tag_tracks([], timestamp=time.time(), valid=False, force_remove=True)
        return self.status()

    def process_once(self) -> Dict[str, Any]:
        with self.process_lock:
            jpeg = self.camera.get_jpeg()
            if not jpeg:
                result = {"ok": False, "error": "no_camera_frame", "detections": []}
            else:
                snapshot = self.scene.snapshot()
                result = self.localizer.localize(
                    jpeg, snapshot.get("camera") or {}, snapshot.get("calibration") or {},
                    registered_parts=snapshot.get("registeredParts") or [],
                    registered_bins=snapshot.get("registeredBins") or [],
                    support_surfaces=snapshot.get("supportSurfaces") or [],
                )
                observed_part_ids = [
                    str(item.get("partId")) for item in (result.get("visibleTags") or [])
                    if item.get("bound") and item.get("partId")
                ]
                rejection_by_part = {
                    str(item.get("partId")): item
                    for item in ((result.get("quality") or {}).get("objectTagRejections") or [])
                    if item.get("partId")
                }
                observed_bin_ids = [
                    str(item.get("binId")) for item in (result.get("visibleTags") or [])
                    if item.get("bound") and item.get("bindingKind") == "bin" and item.get("binId")
                ]
                rejection_by_bin = {
                    str(item.get("binId")): item
                    for item in ((result.get("quality") or {}).get("objectTagRejections") or [])
                    if item.get("binId")
                }
                if result.get("ok"):
                    tracks = self.scene.ingest_tag_tracks(
                        result.get("detections") or [], timestamp=time.time(), valid=True,
                        observed_part_ids=observed_part_ids, rejection_by_part=rejection_by_part,
                        observed_bin_ids=observed_bin_ids, rejection_by_bin=rejection_by_bin,
                    )
                    result["accepted"] = len(tracks.get("parts") or []) + len(tracks.get("bins") or [])
            if not result.get("ok"):
                self.scene.ingest_tag_tracks(
                    [], timestamp=time.time(), valid=False,
                    observed_part_ids=[
                        str(item.get("partId")) for item in (result.get("visibleTags") or [])
                        if item.get("bound") and item.get("partId")
                    ],
                    rejection_by_part={
                        str(item.get("partId")): item for item in (result.get("visibleTags") or [])
                        if item.get("bound") and item.get("partId")
                    },
                    observed_bin_ids=[
                        str(item.get("binId")) for item in (result.get("visibleTags") or [])
                        if item.get("bound") and item.get("bindingKind") == "bin" and item.get("binId")
                    ],
                    rejection_by_bin={
                        str(item.get("binId")): {**item, "reason": item.get("reason") or item.get("rejectionReason")} for item in (result.get("visibleTags") or [])
                        if item.get("bound") and item.get("bindingKind") == "bin" and item.get("binId")
                    },
                )
        with self.lock:
            self.last_debug_jpeg = result.pop("debugJpeg", None)
            self.last_visible_tags = deepcopy(result.get("visibleTags") or [])
            self.last_result = result
            self.processed_frames += 1
        return result

    def _loop(self) -> None:
        while not self.stop_event.is_set():
            snapshot = self.scene.snapshot()
            config = (snapshot.get("camera") or {}).get("localization") or {}
            calibration = snapshot.get("calibration") or {}
            verification = calibration.get("verification") or {}
            verified = bool(verification.get("passed") or verification.get("testingBypass"))
            intrinsics = calibration.get("intrinsics") or {}
            intrinsic_ok = bool(
                intrinsics.get("ok") and float(intrinsics.get("intrinsicRmsPx") or float("inf")) <= MAX_INTRINSIC_RMS_PX and
                float(intrinsics.get("maximumViewErrorPx") or intrinsics.get("intrinsicRmsPx") or float("inf")) <= MAX_INTRINSIC_VIEW_ERROR_PX
            )
            pose_locked = bool((calibration.get("fiducials") or {}).get("baselineHomography"))
            if config.get("enabled") and intrinsic_ok and pose_locked and verified:
                try:
                    self.process_once()
                except Exception as exc:
                    self.scene.ingest_tag_tracks([], timestamp=time.time(), valid=False)
                    with self.lock:
                        self.last_result = {"ok": False, "error": str(exc), "detections": []}
            elif config.get("enabled"):
                error = "passing_intrinsic_calibration_required" if not intrinsic_ok else "locked_camera_pose_required" if not pose_locked else "nine_point_verification_required"
                self.scene.ingest_tag_tracks([], timestamp=time.time(), valid=False)
                with self.lock:
                    self.last_result = {"ok": False, "error": error, "detections": []}
            self.stop_event.wait(max(0.05, float(config.get("intervalS") or 0.08)))

    def status(self) -> Dict[str, Any]:
        with self.lock:
            result = dict(self.last_result)
            result.pop("detections", None)
            return {"running": bool(self.thread and self.thread.is_alive()), "processedFrames": self.processed_frames, **result}

    def get_debug_jpeg(self) -> Optional[bytes]:
        with self.lock:
            return self.last_debug_jpeg

    def visible_tags(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "ok": bool(self.last_result.get("ok")),
                "error": self.last_result.get("error"),
                "frameSize": deepcopy(self.last_result.get("frameSize")),
                "quality": deepcopy(self.last_result.get("quality") or {}),
                "tags": deepcopy(self.last_visible_tags),
                "timestamp": time.time(),
            }
