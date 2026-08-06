import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from fiducial_localization import FiducialLocalizer, marker_robot_corners
from workcell import Workcell


class TaggedBinRegistryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cell = Workcell(Path(self.temp.name))
        payload = self.cell.upsert_bin({
            "id": "bin-a", "label": "Bin A",
            "position": {"x": 0.20, "y": -0.10, "z": 0.0},
            "outer": {"x": 0.14, "y": 0.12, "z": 0.04},
        })
        self.assertTrue(payload["ok"])

    def tearDown(self):
        self.temp.cleanup()

    def bind(self, **extra):
        return self.cell.bind_tagged_bin({
            "binId": "bin-a", "tagId": 11,
            "tagOffsetM": {"x": 0.02, "y": -0.01},
            "mountHeightM": 0.04, "yawOffsetDeg": 90,
            **extra,
        })

    def detection(self, x=0.24, y=-0.08, surface="surface-table"):
        return {
            "id": "bin-a", "localizationSource": "bin_tag", "tagId": 11,
            "position": {"x": x, "y": y, "z": 0.0}, "orientationDeg": 25.0,
            "supportSurfaceId": surface, "supportSurfaceName": "Main Table",
            "supportSurfaceZ": 0.0, "measuredTagTopZ": 0.041,
            "measuredSupportZ": 0.001, "supportHeightResidualM": 0.001,
            "poseQuality": {"surfaceConfidence": 0.95},
        }

    def test_binding_persists_and_uses_shared_tag_namespace(self):
        result = self.bind()
        self.assertTrue(result["ok"])
        reloaded = Workcell(Path(self.temp.name))
        self.assertEqual(reloaded.registered_bins["bin-a"]["tagId"], 11)
        self.assertAlmostEqual(reloaded.registered_bins["bin-a"]["mountHeightM"], 0.04)

        reloaded.bind_tagged_part({"partId": "part-a", "tagId": 12, "label": "Part A"})
        conflict = reloaded.bind_tagged_bin({"binId": "bin-a", "tagId": 12})
        self.assertFalse(conflict["ok"])
        self.assertEqual(conflict["conflictingKind"], "part")
        reassigned = reloaded.bind_tagged_bin({"binId": "bin-a", "tagId": 12, "reassign": True})
        self.assertTrue(reassigned["ok"])
        self.assertNotIn("part-a", reloaded.registered_parts)

    def test_tracking_freshness_retention_and_same_id_restoration(self):
        self.bind()
        started = time.time()
        tracks = self.cell.ingest_tag_tracks([self.detection()], timestamp=started)
        self.assertEqual(tracks["bins"][0]["id"], "bin-a")
        self.assertTrue(tracks["bins"][0]["poseFresh"])

        degraded = self.cell.ingest_tag_tracks(
            [], timestamp=started + 0.4, observed_bin_ids=["bin-a"],
            rejection_by_bin={"bin-a": {"reason": "support_surface_stabilizing"}},
        )
        self.assertEqual(degraded["bins"][0]["trackingState"], "degraded_recent")
        self.assertTrue(degraded["bins"][0]["poseFresh"])

        stale = self.cell.ingest_tag_tracks([], timestamp=started + 1.1)
        self.assertFalse(stale["bins"][0]["poseFresh"])
        self.assertTrue(stale["bins"][0]["displayVisible"])
        removed = self.cell.ingest_tag_tracks([], timestamp=started + 2.5)
        self.assertEqual(removed["bins"], [])
        self.assertEqual(removed["removedBinIds"], ["bin-a"])

        restored = self.cell.ingest_tag_tracks([self.detection(0.25, -0.07)], timestamp=started + 2.6)
        self.assertEqual(restored["bins"][0]["id"], "bin-a")
        self.assertAlmostEqual(self.cell.bins["bin-a"]["position"]["x"], 0.25)

    def test_unbind_keeps_pose_but_requires_physical_confirmation(self):
        self.bind()
        self.cell.ingest_tag_tracks([self.detection()], timestamp=time.time())
        result = self.cell.unbind_tagged_bin("bin-a")
        self.assertTrue(result["ok"])
        bin_obj = self.cell.bins["bin-a"]
        self.assertEqual(bin_obj["positionStatus"], "simulation_only")
        self.assertNotIn("bin-a", self.cell.registered_bins)
        self.assertAlmostEqual(bin_obj["position"]["x"], 0.24)

    def test_delete_removes_registration(self):
        self.bind()
        self.cell.delete_bin("bin-a")
        self.assertNotIn("bin-a", self.cell.bins)
        self.assertNotIn("bin-a", self.cell.registered_bins)

    def test_stale_tracked_bin_is_omitted_from_spatial_context(self):
        self.bind()
        context = self.cell.spatial_context()
        self.assertFalse(any(item["id"] == "bin-a" for item in context["entities"]))
        self.cell.ingest_tag_tracks([self.detection()], timestamp=time.time())
        context = self.cell.spatial_context()
        self.assertTrue(any(item["id"] == "bin-a" for item in context["entities"]))

    def test_execution_snapshot_rejects_stale_or_moved_tracked_bin(self):
        self.bind()
        now = time.time()
        self.cell.ingest_tag_tracks([self.detection()], timestamp=now)
        plan = {"destinationSnapshots": [{
            "kind": "bin", "id": "bin-a", "position": {"x": 0.24, "y": -0.08},
            "trackingMode": "apriltag", "supportSurfaceId": "surface-table",
        }]}
        self.assertIsNone(self.cell.validate_plan_object_snapshots(plan))
        self.cell.ingest_tag_tracks([self.detection(0.246, -0.08)], timestamp=now + 0.1)
        self.assertIn("moved after planning", self.cell.validate_plan_object_snapshots(plan))
        self.cell.ingest_tag_tracks([], timestamp=now + 1.2)
        with patch("workcell.time.time", return_value=now + 1.2):
            self.assertIn("fresh AprilTag pose", self.cell.validate_plan_object_snapshots(plan))


class TaggedBinLocalizationTests(unittest.TestCase):
    def test_rim_tag_recovers_bin_center_yaw_and_raised_surface(self):
        localizer = FiducialLocalizer()
        matrix = np.asarray([[800.0, 0.0, 400.0], [0.0, 800.0, 300.0], [0.0, 0.0, 1.0]])
        rotation = np.diag([1.0, -1.0, -1.0])
        rvec, _ = cv2.Rodrigues(rotation)
        tvec = np.asarray([[0.0], [0.0], [0.6]])
        center = np.asarray([0.20, -0.04])
        offset = np.asarray([0.03, -0.01])
        yaw = 90.0
        angle = np.radians(yaw)
        rotate = np.asarray([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        tag_center = center + rotate @ offset
        robot_xy = marker_robot_corners({
            "center": {"x": tag_center[0], "y": tag_center[1]},
            "sizeM": 0.03, "yawDeg": yaw,
        }, 0.03)
        # Platform top 80 mm + rigid rim mount 45 mm.
        pixels, _ = cv2.projectPoints(
            np.column_stack((robot_xy, np.full(4, 0.125))), rvec, tvec, matrix, np.zeros(5)
        )
        mapping = {11: {
            "binId": "bin-a", "entityKind": "bin", "entityId": "bin-a",
            "tagSizeM": 0.03, "outer": {"x": 0.16, "y": 0.12, "z": 0.045},
            "mountHeightM": 0.045, "tagOffsetM": {"x": offset[0], "y": offset[1]},
        }}
        surfaces = [{
            "id": "platform", "name": "Platform", "center": {"x": 0.20, "y": -0.04},
            "size": {"x": 0.40, "y": 0.30}, "topZ": 0.08,
            "entryToleranceM": 0.015, "holdToleranceM": 0.020, "enabled": True,
        }]
        detections = []
        for _ in range(3):
            detections = localizer._tagged_detections(
                [pixels.reshape(1, 4, 2)], [11], mapping, matrix, rvec, tvec, {}, surfaces,
            )
        self.assertEqual(len(detections), 1)
        found = detections[0]
        self.assertEqual(found["localizationSource"], "bin_tag")
        self.assertEqual(found["supportSurfaceId"], "platform")
        self.assertAlmostEqual(found["position"]["x"], center[0], places=4)
        self.assertAlmostEqual(found["position"]["y"], center[1], places=4)
        self.assertAlmostEqual(found["position"]["z"], 0.08, places=6)
        self.assertAlmostEqual(found["orientationDeg"], yaw, places=4)


class TaggedBinUiContractTests(unittest.TestCase):
    def test_tracked_position_status_is_not_inserted_into_three_column_grid(self):
        source = (Path(__file__).resolve().parents[1] / "static" / "js" / "ui.js").read_text()
        self.assertIn("position.body.append(tracked);", source)
        self.assertNotIn("positionGrid.append(tracked);", source)
        self.assertIn("is providing a fresh physical position and rotation", source)


if __name__ == "__main__":
    unittest.main()
