import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class DashboardModuleTests(unittest.TestCase):
    def test_every_module_uses_one_shared_store_instance(self):
        versions = set()
        importers = []
        for path in (ROOT / "static" / "js").glob("*.js"):
            matches = re.findall(r'store\.js\?v=(\d+)', path.read_text())
            if matches:
                importers.append(path.name)
                versions.update(matches)
        self.assertGreaterEqual(len(importers), 4)
        self.assertEqual(
            len(versions), 1,
            f"Different store.js query versions create isolated state objects: {sorted(versions)}",
        )

    def test_every_module_uses_one_initialized_viewport_instance(self):
        versions = set()
        importers = []
        for path in (ROOT / "static" / "js").glob("*.js"):
            matches = re.findall(r'viewport\.js\?v=(\d+)', path.read_text())
            if matches:
                importers.append(path.name)
                versions.update(matches)
        self.assertGreaterEqual(len(importers), 3)
        self.assertEqual(
            len(versions), 1,
            f"Different viewport.js query versions split the initialized path layer: {sorted(versions)}",
        )

    def test_robot_and_environment_share_one_metric_scale(self):
        source = (ROOT / "static" / "js" / "viewport.js").read_text()
        self.assertIn("const scale = SCENE_METERS_TO_UNITS;", source)
        self.assertNotIn("4.15 / Math.max", source)

    def test_held_object_follows_rendered_jaw_not_requested_pose(self):
        source = (ROOT / "static" / "js" / "viewport.js").read_text()
        self.assertIn("const tcp = sceneToRobotFrame(getGripperTcpScenePosition());", source)
        self.assertNotIn("const tcp = coordinatePose || sceneToRobotFrame", source)

    def test_renderer_uses_firmware_base_frame_and_fixed_jaw_tcp(self):
        source = (ROOT / "static" / "js" / "viewport.js").read_text()
        self.assertIn("fitted.position.copy(robotFrameToScene(FIRMWARE_BASE_TRANSLATION_M));", source)
        self.assertNotIn("GRIPPER_GRASP_CLOSE_ADVANCE_Y", source)

    def test_pick_depth_diagnostics_are_visible_and_legacy_lift_is_gone(self):
        ui = (ROOT / "static" / "js" / "ui.js").read_text()
        viewport = (ROOT / "static" / "js" / "viewport.js").read_text()
        html = (ROOT / "static" / "index.html").read_text()
        self.assertIn("Pick depth", ui)
        self.assertIn("plannedHeightModel", viewport)
        self.assertIn("renderedFingertipLowZ", viewport)
        self.assertNotIn("toolVerticalLiftInput", html)
        self.assertIn("minimumTableClearanceInput", html)
        self.assertNotIn("toolVerticalLiftM", ui)

    def test_automatic_camera_objects_and_legacy_offset_routes_are_absent(self):
        server = (ROOT / "web_server.py").read_text()
        workcell = (ROOT / "workcell.py").read_text()
        self.assertNotIn('"/api/camera/detections"', server)
        self.assertNotIn('"/api/camera/accept-detections"', server)
        self.assertNotIn("def ingest_detections", workcell)
        self.assertNotIn("def accept_scan_detections", workcell)
        self.assertNotIn("def suppress_camera_tracks", workcell)
        self.assertIn("classify_visible_part", server)

    def test_official_suction_cad_is_fixed_to_base_and_head_follows_flange(self):
        viewport = (ROOT / "static" / "js" / "viewport.js").read_text()
        vendor = ROOT / "static" / "vendor" / "suction_gripper"
        self.assertTrue((vendor / "pump_box.dae").is_file())
        self.assertTrue((vendor / "pump_head.dae").is_file())
        self.assertIn('"pump_box.dae", [0, -0.15, 0]', viewport)
        self.assertIn('"pump_head.dae", [0, -0.008, 0]', viewport)
        self.assertLess(viewport.index('"pump_box.dae"'), viewport.index("const joint1 = makeJointFrame"))
        self.assertIn("SUCTION_CONTACT_DISTANCE_M = 0.072", viewport)
        self.assertIn("SUCTION_MOUNT_TRANSLATION_M = 0.010", viewport)
        self.assertIn("SUCTION_FACE_CLOCKING_RAD = Math.PI / 2", viewport)
        self.assertIn("applySuctionMountRotation(gripper)", viewport)
        attribution = (vendor / "README.md").read_text()
        self.assertIn("BSD 3-Clause", attribution)
        self.assertIn("physical center of mass is therefore", attribution)

    def test_official_suction_cad_units_axes_and_raw_bounds(self):
        vendor = ROOT / "static" / "vendor" / "suction_gripper"

        def bounds(name):
            root = ET.parse(vendor / name).getroot()
            unit = next(element for element in root.iter() if element.tag.endswith("unit"))
            up_axis = next(element for element in root.iter() if element.tag.endswith("up_axis"))
            self.assertEqual(float(unit.attrib["meter"]), 0.001)
            self.assertEqual((up_axis.text or "").strip(), "Z_UP")
            vertices = []
            for element in root.iter():
                if element.tag.endswith("float_array") and "position" in element.attrib.get("id", "").lower():
                    values = [float(value) for value in (element.text or "").split()]
                    vertices.extend(zip(values[0::3], values[1::3], values[2::3]))
            dimensions = [
                (max(point[axis] for point in vertices) - min(point[axis] for point in vertices)) * 0.001
                for axis in range(3)
            ]
            return sorted(dimensions)

        box = bounds("pump_box.dae")
        head = bounds("pump_head.dae")
        for actual, expected in zip(box, [0.043, 0.052, 0.072]):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(head, [0.0245, 0.0267, 0.063]):
            self.assertAlmostEqual(actual, expected, places=6)
        license_text = (vendor / "LICENSE.BSD-3-Clause").read_text()
        self.assertIn("Copyright (c) 2023, [fullname]", license_text)

    def test_tool_and_part_pickup_calibration_controls_are_exposed(self):
        ui = (ROOT / "static" / "js" / "ui.js").read_text()
        html = (ROOT / "static" / "index.html").read_text()
        self.assertIn("Tool Contact Calibration", html)
        self.assertIn("Pickup Setup", ui)
        self.assertIn("observedContactMissMm", ui)
        self.assertIn("pin 5 is Pump; pin 2 is Release Valve", html)


if __name__ == "__main__":
    unittest.main()
