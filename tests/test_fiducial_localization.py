import math
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from fiducial_localization import (
    DICTIONARY_NAME,
    CharucoCalibrationSession,
    FiducialLocalizer,
    TrackStabilizer,
    apply_homography,
    aruco_dictionary,
    encode_jpeg,
    marker_robot_corners,
    verification_report,
)
from workcell import Workcell


def synthetic_workspace(marker_ids=(0, 1, 2, 3)):
    image = np.full((600, 800, 3), 255, np.uint8)
    placements = {0: (70, 60), 1: (650, 60), 2: (650, 460), 3: (70, 460)}
    size = 80
    dictionary = aruco_dictionary()
    reference = []
    for marker_id in marker_ids:
        x, y = placements[marker_id]
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, size)
        image[y:y + size, x:x + size] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        pixels = [(x, y), (x + size - 1, y), (x + size - 1, y + size - 1), (x, y + size - 1)]
        reference.append({
            "id": marker_id,
            "corners": [{"x": (600 - py) * 0.0005, "y": (400 - px) * 0.0005} for px, py in pixels],
        })
    cv2.rectangle(image, (350, 260), (430, 330), (0, 0, 0), -1)
    calibration = {
        "intrinsics": {
            "cameraMatrix": [[800, 0, 400], [0, 800, 300], [0, 0, 1]],
            "distortionCoefficients": [0, 0, 0, 0, 0],
        },
        "fiducials": {
            "dictionary": DICTIONARY_NAME,
            "referenceMarkers": reference,
            "minimumCoverageRatio": 0.1,
        },
    }
    camera = {
        "workspaceBounds": {"xMin": 0, "xMax": 0.35, "yMin": -0.25, "yMax": 0.25},
    }
    return encode_jpeg(image, 100), camera, calibration


def use_center_marker_map(calibration):
    for marker in calibration["fiducials"]["referenceMarkers"]:
        corners = marker.pop("corners")
        marker["center"] = {"x": sum(p["x"] for p in corners) / 4, "y": sum(p["y"] for p in corners) / 4}
        marker["sizeM"] = 79 * 0.0005
        marker["yawDeg"] = 0
    return calibration


class FiducialLocalizationTests(unittest.TestCase):
    def test_marker_robot_corner_order_and_positive_yaw(self):
        marker = {"center": {"x": 0.3, "y": 0.1}, "sizeM": 0.05, "yawDeg": 0}
        expected = np.asarray([[0.325, 0.125], [0.325, 0.075], [0.275, 0.075], [0.275, 0.125]])
        np.testing.assert_allclose(marker_robot_corners(marker, 0.05), expected)
        rotations = {
            90: [[0.275, 0.125], [0.325, 0.125], [0.325, 0.075], [0.275, 0.075]],
            -90: [[0.325, 0.075], [0.275, 0.075], [0.275, 0.125], [0.325, 0.125]],
            180: [[0.275, 0.075], [0.275, 0.125], [0.325, 0.125], [0.325, 0.075]],
        }
        for yaw, corners in rotations.items():
            marker["yawDeg"] = yaw
            np.testing.assert_allclose(marker_robot_corners(marker, 0.05), corners, atol=1e-9)

    def test_explicit_marker_corners_remain_backward_compatible(self):
        corners = [{"x": 1, "y": 2}, {"x": 3, "y": 4}, {"x": 5, "y": 6}, {"x": 7, "y": 8}]
        np.testing.assert_array_equal(marker_robot_corners({"corners": corners}, 0.05), [[1, 2], [3, 4], [5, 6], [7, 8]])

    def test_charuco_capture_ignores_duplicate_workspace_marker_ids(self):
        session = CharucoCalibrationSession()
        image = np.full((720, 1280, 3), 255, np.uint8)
        board = session.board.generateImage((560, 400), marginSize=0, borderBits=1)
        image[160:560, 360:920] = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
        for marker_id, (x, y) in enumerate(((80, 60), (1060, 60), (1060, 560), (80, 560))):
            marker = cv2.aruco.generateImageMarker(session.dictionary, marker_id, 110)
            image[y:y + 110, x:x + 110] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        result = session.capture(encode_jpeg(image))
        self.assertTrue(result["ok"], result)
        self.assertGreaterEqual(result["cornerCount"], 8)
        self.assertGreater(result["detectedMarkerCount"], result["markerCount"])

    def test_exact_homography_projection(self):
        h = np.asarray([[0, -0.0005, 0.3], [-0.0005, 0, 0.2], [0, 0, 1]], np.float64)
        self.assertEqual(apply_homography(h, (400, 300)), (0.15, 0.0))

    def test_synthetic_workspace_localizes_without_base_estimation(self):
        jpeg, camera, calibration = synthetic_workspace()
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["quality"]["visibleMarkerCount"], 4)
        self.assertLess(result["quality"]["reprojectionRmsPx"], 1.0)
        self.assertTrue(result.get("debugJpeg"))
        self.assertEqual(result["detections"], [])

    def test_center_based_physical_layout_uses_correct_corner_correspondence(self):
        jpeg, camera, calibration = synthetic_workspace()
        use_center_marker_map(calibration)
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["quality"]["inlierCornerCount"], 16)
        self.assertTrue(all(marker["passed"] for marker in result["quality"]["perMarker"]))

    def test_swapped_marker_ids_are_rejected_as_invalid_layout(self):
        jpeg, camera, calibration = synthetic_workspace()
        use_center_marker_map(calibration)
        markers = calibration["fiducials"]["referenceMarkers"]
        markers[0]["center"], markers[1]["center"] = markers[1]["center"], markers[0]["center"]
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertEqual(result["error"], "reference_layout_invalid")

    def test_reversed_y_signs_are_rejected_as_invalid_layout(self):
        jpeg, camera, calibration = synthetic_workspace()
        use_center_marker_map(calibration)
        for marker in calibration["fiducials"]["referenceMarkers"]:
            marker["center"]["y"] *= -1
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertEqual(result["error"], "reference_layout_invalid")

    def test_duplicate_and_near_collinear_centers_are_rejected(self):
        jpeg, camera, calibration = synthetic_workspace()
        use_center_marker_map(calibration)
        markers = calibration["fiducials"]["referenceMarkers"]
        markers[1]["center"] = dict(markers[0]["center"])
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertEqual(result["error"], "reference_layout_invalid")
        self.assertTrue(result["quality"]["referenceLayoutErrors"])

    def test_wrong_marker_yaw_reports_specific_failing_marker(self):
        jpeg, camera, calibration = synthetic_workspace()
        marker = calibration["fiducials"]["referenceMarkers"][0]
        corners = marker.pop("corners")
        marker["center"] = {"x": sum(p["x"] for p in corners) / 4, "y": sum(p["y"] for p in corners) / 4}
        marker["sizeM"] = 79 * 0.0005
        marker["yawDeg"] = 90
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertFalse(result["ok"])
        self.assertIn(0, result["quality"]["failingMarkerIds"])
        self.assertIn(result["error"], ("marker_inliers_insufficient", "reprojection_error_excessive"))

    def test_marker_occlusion_rejects_frame(self):
        jpeg, camera, calibration = synthetic_workspace((0, 1))
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertFalse(result["ok"])
        self.assertEqual(result["error"], "insufficient_reference_markers")

    def test_unknown_ids_do_not_count_as_reference(self):
        jpeg, camera, calibration = synthetic_workspace((0, 1, 2))
        calibration["fiducials"]["referenceMarkers"] = calibration["fiducials"]["referenceMarkers"][:2]
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertFalse(result["ok"])
        self.assertEqual(result["error"], "unknown_marker_ids")

    def test_poor_conditioning_is_rejected(self):
        jpeg, camera, calibration = synthetic_workspace()
        calibration["fiducials"]["maxConditionNumber"] = 10
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertFalse(result["ok"])
        self.assertEqual(result["error"], "homography_poorly_conditioned")

    def test_out_of_bounds_contour_is_not_published(self):
        jpeg, camera, calibration = synthetic_workspace()
        camera["workspaceBounds"] = {"xMin": 0, "xMax": 0.05, "yMin": 0, "yMax": 0.05}
        result = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertTrue(result["ok"])
        self.assertEqual(result["detections"], [])

    def test_object_tag_provides_identity_position_and_yaw(self):
        jpeg, camera, calibration = synthetic_workspace()
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        marker = cv2.aruco.generateImageMarker(aruco_dictionary(), 10, 70)
        image[170:240, 365:435] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        calibration["fiducials"]["objectTags"] = [{
            "id": 10, "label": "Tagged Cube", "class": "cube",
            "size": {"x": 0.04, "y": 0.04, "z": 0.06}, "yawOffsetDeg": 0,
        }]
        result = FiducialLocalizer().localize(encode_jpeg(image, 100), camera, calibration)
        tagged = [d for d in result["detections"] if d["localizationSource"] == "object_tag"]
        self.assertEqual(len(tagged), 1)
        self.assertEqual(tagged[0]["label"], "Tagged Cube")
        self.assertAlmostEqual(tagged[0]["position"]["z"], 0.03)

    def test_elevated_tag_plane_recovers_offsets_and_wrapped_yaw(self):
        localizer = FiducialLocalizer()
        matrix = np.asarray([[800.0, 0.0, 400.0], [0.0, 800.0, 300.0], [0.0, 0.0, 1.0]])
        rotation = np.diag([1.0, -1.0, -1.0])
        rvec, _ = cv2.Rodrigues(rotation)
        tvec = np.asarray([[0.0], [0.0], [0.6]])
        offset = np.asarray([0.012, -0.006])
        object_center = np.asarray([0.18, 0.07])
        for yaw in (0.0, 90.0, -179.0, 179.0):
            angle = math.radians(yaw)
            rotate = np.asarray([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
            tag_center = object_center + rotate @ offset
            robot_xy = marker_robot_corners({
                "center": {"x": tag_center[0], "y": tag_center[1]},
                "sizeM": 0.03, "yawDeg": yaw,
            }, 0.03)
            robot_xyz = np.column_stack((robot_xy, np.full(4, 0.08)))
            pixels, _ = cv2.projectPoints(robot_xyz, rvec, tvec, matrix, np.zeros(5))
            mapping = {10: {
                "partId": "part-tagged", "tagId": 10, "label": "Offset Box",
                "size": {"x": 0.08, "y": 0.05, "z": 0.08}, "tagSizeM": 0.03,
                "tagOffsetM": {"x": offset[0], "y": offset[1]}, "yawOffsetDeg": 0,
            }}
            detections = localizer._tagged_detections(
                [pixels.reshape(1, 4, 2)], [10], mapping, matrix, rvec, tvec,
                {"visibleMarkerCount": 4, "reprojectionRmsPx": 0.0},
            )
            self.assertEqual(len(detections), 1)
            self.assertAlmostEqual(detections[0]["position"]["x"], object_center[0], places=6)
            self.assertAlmostEqual(detections[0]["position"]["y"], object_center[1], places=6)
            self.assertAlmostEqual(detections[0]["position"]["z"], 0.04, places=6)
            angular_error = (detections[0]["orientationDeg"] - yaw + 180.0) % 360.0 - 180.0
            self.assertAlmostEqual(angular_error, 0.0, places=6)

    def test_physically_wrong_object_tag_size_is_rejected(self):
        localizer = FiducialLocalizer()
        matrix = np.asarray([[800.0, 0.0, 400.0], [0.0, 800.0, 300.0], [0.0, 0.0, 1.0]])
        rotation = np.diag([1.0, -1.0, -1.0])
        rvec, _ = cv2.Rodrigues(rotation)
        tvec = np.asarray([[0.0], [0.0], [0.6]])
        robot_xy = marker_robot_corners({"center": {"x": 0.18, "y": 0.07}, "sizeM": 0.06}, 0.06)
        pixels, _ = cv2.projectPoints(np.column_stack((robot_xy, np.full(4, 0.05))), rvec, tvec, matrix, np.zeros(5))
        mapping = {10: {"partId": "wrong-tag", "tagSizeM": 0.03, "size": {"x": 0.08, "y": 0.08, "z": 0.05}}}
        self.assertEqual(localizer._tagged_detections(
            [pixels.reshape(1, 4, 2)], [10], mapping, matrix, rvec, tvec,
            {"visibleMarkerCount": 4, "reprojectionRmsPx": 0.0},
        ), [])

    def test_unbound_object_tag_is_selectable_but_not_a_scene_detection(self):
        jpeg, camera, calibration = synthetic_workspace()
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        marker = cv2.aruco.generateImageMarker(aruco_dictionary(), 10, 60)
        image[180:240, 370:430] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        result = FiducialLocalizer().localize(encode_jpeg(image, 100), camera, calibration)
        self.assertTrue(result["ok"], result)
        candidate = next(item for item in result["visibleTags"] if item["tagId"] == 10)
        self.assertFalse(candidate["bound"])
        self.assertTrue(math.isfinite(candidate["robotTablePosition"]["x"]))
        self.assertTrue(math.isfinite(candidate["robotTablePosition"]["y"]))
        self.assertEqual(result["detections"], [])

    def test_tag_registry_disappears_and_restores_without_frame_writes(self):
        with tempfile.TemporaryDirectory() as folder:
            cell = Workcell(Path(folder))
            bound = cell.bind_tagged_part({
                "tagId": 10, "label": "Blue Box", "type": "box",
                "size": {"x": 0.05, "y": 0.04, "z": 0.03},
            })
            part_id = bound["registeredPart"]["partId"]
            detection = {
                "id": part_id, "localizationSource": "object_tag", "orientationDeg": 15,
                "position": {"x": 0.2, "y": 0.1, "z": 0.015}, "bboxPx": {"x": 1, "y": 2, "width": 30, "height": 30},
            }
            with patch.object(cell, "_save_locked") as save:
                cell.ingest_tag_tracks([detection], timestamp=10.0)
                save.assert_not_called()
                self.assertIn(part_id, cell.parts)
                membership_version = cell.version
                cell.ingest_tag_tracks([{**detection, "position": {"x": 0.201, "y": 0.1, "z": 0.015}}], timestamp=10.1)
                self.assertEqual(cell.version, membership_version)
                cell.ingest_tag_tracks([], timestamp=10.5)
                self.assertIn(part_id, cell.parts)
                cell.ingest_tag_tracks([], timestamp=11.1)
                self.assertNotIn(part_id, cell.parts)
                self.assertIn(part_id, cell.registered_parts)
                hidden = next(item for item in cell.snapshot()["registeredParts"] if item["partId"] == part_id)
                self.assertFalse(hidden["visible"])
                self.assertNotIn("position", hidden)
                cell.ingest_tag_tracks([detection], timestamp=12.0)
                self.assertEqual(cell.parts[part_id]["label"], "Blue Box")

    def test_tag_binding_requires_explicit_reassignment_and_unbinds_to_virtual(self):
        with tempfile.TemporaryDirectory() as folder:
            cell = Workcell(Path(folder))
            first = cell.bind_tagged_part({"tagId": 10, "label": "First"})["registeredPart"]
            rejected = cell.bind_tagged_part({"tagId": 10, "label": "Second"})
            self.assertTrue(rejected["requiresReassign"])
            second = cell.bind_tagged_part({"tagId": 10, "label": "Second", "reassign": True})["registeredPart"]
            self.assertNotEqual(first["partId"], second["partId"])
            result = cell.unbind_tagged_part(second["partId"])
            self.assertEqual(result["part"]["trackingMode"], "virtual")
            self.assertNotIn(second["partId"], cell.registered_parts)

    def test_moved_camera_is_rejected(self):
        jpeg, camera, calibration = synthetic_workspace()
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        object_tag = cv2.aruco.generateImageMarker(aruco_dictionary(), 10, 70)
        image[155:225, 365:435] = cv2.cvtColor(object_tag, cv2.COLOR_GRAY2BGR)
        jpeg = cv2.imencode(".jpg", image)[1].tobytes()
        first = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertTrue(first["ok"])
        baseline = np.asarray(first["homography"])
        baseline[0, 2] += 0.02
        calibration["fiducials"]["baselineHomography"] = baseline.tolist()
        moved = FiducialLocalizer().localize(jpeg, camera, calibration)
        self.assertFalse(moved["ok"])
        self.assertEqual(moved["error"], "camera_moved_reaccept_required")
        self.assertEqual(moved["detections"], [])
        self.assertEqual(moved["frameSize"], {"width": 800, "height": 600})
        self.assertIn(10, [tag["tagId"] for tag in moved["visibleTags"]])
        self.assertNotIn("robotTablePosition", next(tag for tag in moved["visibleTags"] if tag["tagId"] == 10))

    def test_tag_tracker_uses_three_frame_median_and_wrapped_yaw(self):
        tracker = TrackStabilizer()
        final = None
        for index, (x, yaw) in enumerate(((0.1000, 179.0), (0.1010, -179.0), (0.0995, 178.0))):
            final = tracker.update([{
                "id": "tag-part-10", "trackId": "tag-part-10",
                "position": {"x": x, "y": 0.2, "z": 0.03},
                "orientationDeg": yaw, "localizationSource": "object_tag",
            }], now=index + 1)[0]
        self.assertEqual(final["trackId"], "tag-part-10")
        self.assertAlmostEqual(final["position"]["x"], 0.1000, places=6)
        self.assertLessEqual(final["stabilitySpreadM"], 0.002)
        self.assertLess(abs(abs(final["orientationDeg"]) - 180.0), 2.0)

    def test_nine_point_verification_thresholds(self):
        samples = [
            {"expected": {"x": i * 0.02, "y": j * 0.02}, "measured": {"x": i * 0.02 + 0.001, "y": j * 0.02 - 0.001}}
            for i in range(3) for j in range(3)
        ]
        report = verification_report(samples, stationary_spread_m=0.001)
        self.assertTrue(report["passed"])
        self.assertLess(report["rmsXyErrorM"], 0.003)

    def test_nine_point_verification_requires_stationary_spread(self):
        samples = [{"expected": {"x": i, "y": 0}, "measured": {"x": i, "y": 0}} for i in range(9)]
        self.assertFalse(verification_report(samples)["passed"])
        self.assertFalse(verification_report(samples, stationary_spread_m=0.0051)["passed"])
        self.assertTrue(verification_report(samples, stationary_spread_m=0.005)["passed"])

    def test_charuco_remove_last_and_diversity_report(self):
        session = CharucoCalibrationSession()
        session.samples = [(np.zeros((8, 1, 3), np.float32), np.zeros((8, 1, 2), np.float32))] * 12
        session.sample_signatures = [
            (0.5, 0.5, 0.2, 0.2, 0.1), (0.2, 0.2, 0.15, 0.15, 0.1),
            (0.8, 0.2, 0.3, 0.3, 0.1), (0.2, 0.8, 0.5, 0.5, 0.1),
            (0.8, 0.8, 0.2, 0.2, 0.0), (0.5, 0.2, 0.3, 0.3, 0.0),
            (0.2, 0.5, 0.5, 0.5, 0.0), (0.8, 0.5, 0.2, 0.2, 0.0),
            (0.5, 0.8, 0.3, 0.3, 0.0), (0.4, 0.4, 0.5, 0.5, 0.0),
            (0.6, 0.4, 0.2, 0.2, 0.0), (0.4, 0.6, 0.3, 0.3, 0.0),
        ]
        session.sample_quality = [{"index": i + 1} for i in range(12)]
        session.image_size = (1280, 720)
        self.assertTrue(session.status()["diversity"]["passed"])
        session.remove_last()
        self.assertEqual(session.status()["sampleCount"], 11)

    def test_charuco_diversity_accepts_two_distinct_scales(self):
        session = CharucoCalibrationSession()
        session.sample_signatures = [
            (0.5, 0.5, 0.2, 0.2, 0.1),
            (0.2, 0.2, 0.4, 0.3, 0.1),
            (0.8, 0.2, 0.2, 0.2, 0.1),
            (0.2, 0.8, 0.4, 0.3, 0.1),
            (0.8, 0.8, 0.2, 0.2, 0.0),
        ]
        report = session.status()["diversity"]
        self.assertTrue(report["passed"])
        self.assertEqual(report["scaleLevelCount"], 2)

    def test_charuco_diversity_names_missing_second_scale(self):
        session = CharucoCalibrationSession()
        session.sample_signatures = [
            (0.5, 0.5, 0.2, 0.2, 0.1),
            (0.2, 0.2, 0.2, 0.2, 0.1),
            (0.8, 0.2, 0.2, 0.2, 0.1),
            (0.2, 0.8, 0.2, 0.2, 0.1),
            (0.8, 0.8, 0.2, 0.2, 0.0),
        ]
        report = session.status()["diversity"]
        self.assertFalse(report["passed"])
        self.assertIn("second_board_scale", report["missing"])
        self.assertNotIn("three_board_scales", report["missing"])

    def test_intrinsic_solution_rejects_bad_per_view_error(self):
        session = CharucoCalibrationSession()
        session.samples = [(np.zeros((8, 1, 3), np.float32), np.zeros((8, 1, 2), np.float32))] * 12
        session.sample_signatures = [(0.5, 0.5, 0.2, 0.2, 0.1), (0.2, 0.2, 0.15, 0.15, 0.1), (0.8, 0.2, 0.3, 0.3, 0.1), (0.2, 0.8, 0.5, 0.5, 0.1), (0.8, 0.8, 0.2, 0.2, 0.0)] * 3
        session.sample_signatures = session.sample_signatures[:12]
        session.sample_quality = [{"index": i + 1} for i in range(12)]
        session.image_size = (1280, 720)
        calibration_result = (1.0, np.eye(3), np.zeros((1, 5)), [], [], None, None, np.asarray([[1.0]] * 11 + [[5.0]]))
        with patch("fiducial_localization.cv2.calibrateCameraExtended", return_value=calibration_result):
            result = session.solve()
        self.assertFalse(result["ok"])
        self.assertEqual(result["maximumViewErrorPx"], 5.0)

    def test_empty_verification_report_is_strict_json(self):
        report = verification_report([])
        self.assertFalse(report["passed"])
        self.assertIsNone(report["rmsXyErrorM"])
        self.assertIsNone(report["maxXyErrorM"])
        json.dumps(report, allow_nan=False)

    def test_legacy_calibration_loads_with_new_fiducial_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "workcell.json"
            path.write_text(json.dumps({
                "version": 1, "counter": 1, "parts": [], "bins": [], "programs": [],
                "calibration": {"status": "configured", "cameraToRobot": {"position": {"x": 0, "y": 0, "z": 0.5}, "rpyDeg": {"roll": 180, "pitch": 0, "yaw": 0}}},
            }))
            cell = Workcell(Path(folder))
            self.assertEqual(cell.calibration["fiducials"]["dictionary"], DICTIONARY_NAME)
            self.assertIsNone(cell.calibration["intrinsics"])

    def test_legacy_classifier_configuration_is_removed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "workcell.json"
            path.write_text(json.dumps({
                "camera": {
                    "classifier": {"allowedClasses": ["small_box"], "maxObjects": 3},
                    "preferredName": "Old Camera", "preferredIndex": 2,
                    "allowFallbackCameras": False,
                },
            }))
            camera = Workcell(Path(folder)).camera
            self.assertNotIn("classifier", camera)
            self.assertNotIn("preferredName", camera)
            self.assertNotIn("preferredIndex", camera)
            self.assertNotIn("allowFallbackCameras", camera)

    def test_legacy_coordinate_offset_modes_are_removed_on_load(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "workcell.json"
            path.write_text(json.dumps({
                "coordinatePlanner": {
                    "toolOffsetMode": "z_lift",
                    "legacyToolOffsetMode": "z_lift",
                    "toolVerticalLiftM": {"adaptive_gripper": 0.078},
                    "pickHeightBiasM": 0.006,
                },
            }))
            planner = Workcell(Path(folder)).coordinate_planner
            self.assertNotIn("toolOffsetMode", planner)
            self.assertNotIn("legacyToolOffsetMode", planner)
            self.assertNotIn("toolVerticalLiftM", planner)
            self.assertEqual(planner["pickHeightBiasM"], 0.006)

    def test_geometry_change_invalidates_pose_and_verification(self):
        with tempfile.TemporaryDirectory() as folder:
            cell = Workcell(Path(folder))
            cell.calibration["intrinsics"] = {"ok": True, "intrinsicRmsPx": 1.0, "maximumViewErrorPx": 1.2}
            cell.calibration["fiducials"]["referenceMarkers"] = [{"id": 0, "center": {"x": 0.1, "y": 0.1}}]
            cell.calibration["fiducials"]["baselineHomography"] = np.eye(3).tolist()
            cell.calibration["verification"] = {"passed": True}
            cell.set_calibration({"fiducials": {"referenceMarkers": [{"id": 0, "center": {"x": 0.2, "y": 0.1}}]}})
            self.assertIsNone(cell.calibration["fiducials"]["baselineHomography"])
            self.assertIsNone(cell.calibration["verification"])

    def test_saved_intrinsics_are_detached_from_enriched_api_result(self):
        with tempfile.TemporaryDirectory() as folder:
            cell = Workcell(Path(folder))
            result = {"ok": True, "intrinsicRmsPx": 1.0, "maximumViewErrorPx": 1.2}
            saved = cell.set_calibration({"intrinsics": result})
            result["calibration"] = saved["calibration"]
            self.assertNotIn("calibration", cell.calibration["intrinsics"])
            json.dumps(cell.snapshot(), allow_nan=False)

    def test_continuous_localization_requires_all_calibration_gates(self):
        with tempfile.TemporaryDirectory() as folder:
            cell = Workcell(Path(folder))
            blocked = cell.set_camera_config({"localization": {"enabled": True}})
            self.assertFalse(blocked["ok"])
            self.assertFalse(cell.camera["localization"]["enabled"])
            cell.calibration["intrinsics"] = {"ok": True, "intrinsicRmsPx": 1.0, "maximumViewErrorPx": 1.5}
            cell.calibration["fiducials"]["baselineHomography"] = np.eye(3).tolist()
            cell.calibration["verification"] = {"passed": True, "stationarySpreadM": 0.001}
            accepted = cell.set_camera_config({"localization": {"enabled": True}})
            self.assertTrue(accepted["ok"])
            self.assertTrue(cell.camera["localization"]["enabled"])

    def test_testing_bypass_allows_localization_and_warned_physical_programs(self):
        with tempfile.TemporaryDirectory() as folder:
            cell = Workcell(Path(folder))
            cell.calibration["intrinsics"] = {"ok": True, "intrinsicRmsPx": 1.0, "maximumViewErrorPx": 1.5}
            cell.calibration["fiducials"]["baselineHomography"] = np.eye(3).tolist()
            cell.calibration["verification"] = {"passed": False, "testingBypass": True, "mode": "testing_unverified"}
            accepted = cell.set_camera_config({"localization": {"enabled": True}})
            self.assertTrue(accepted["ok"])
            self.assertTrue(cell.camera["localization"]["enabled"])
            self.assertIsNone(cell.physical_program_gate_error())
            self.assertIn("nine-point", cell.physical_program_warning())
            cell.calibration["verification"] = {"passed": True, "testingBypass": False}
            self.assertIsNone(cell.physical_program_gate_error())
            self.assertIsNone(cell.physical_program_warning())


if __name__ == "__main__":
    unittest.main()
