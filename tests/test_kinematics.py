import math
import random
import tempfile
import time
import unittest
from pathlib import Path

import mycobot_kinematics as kin
from web_server import (
    COORD_PHYSICAL_ANGULAR_TOLERANCE_MM,
    COORD_PHYSICAL_RPY_TOLERANCE_DEG,
    COORD_PHYSICAL_TOLERANCE_MM,
    HostKinematicsPreviewRobot,
    RobotService,
    json_safe,
)
from workcell import HOME_ANGLES, Workcell, validate_coordinate_bounds
from mycobot_driver import MyCobotDriver


class KinematicsTests(unittest.TestCase):
    @staticmethod
    def _tool_cell(tool_id="adaptive_gripper"):
        cell = Workcell.__new__(Workcell)
        cell.end_effector = tool_id
        cell.coordinate_planner = Workcell._default_coordinate_planner()
        return cell

    def test_suction_flange_tcp_round_trip_uses_measured_contact_and_correction(self):
        flange = ((0.11, -0.04, 0.19), kin.rotation_from_rpy_deg([20, -30, 70]))
        correction = (0.001, -0.002, 0.003)
        tcp = kin.tcp_from_flange(*flange, "suction_gripper", correction, 0.072)
        recovered = kin.flange_from_tcp(*tcp, "suction_gripper", correction, 0.072)
        self.assertLess(math.dist(flange[0], recovered[0]), 1e-12)
        self.assertLess(kin.pose_residual((0, 0, 0), flange[1], (0, 0, 0), recovered[1])[1], 1e-12)
        vector, _ = kin.tool_transform("suction_gripper")
        # The official mount angle is 1.579 rad rather than a mathematically
        # exact pi/2, so its 10 mm flange translation introduces a sub-micron
        # difference from the measured 72 mm straight-line length.
        self.assertAlmostEqual(math.dist((0, 0, 0), vector), 0.072, places=6)

    def test_suction_head_uses_corrected_rear_view_clocking_on_j6(self):
        expected = kin._mat_mul(kin._rotz(math.pi / 2), kin._rotx(1.579))
        for actual_row, expected_row in zip(kin.SUCTION_TOOL_ROTATION, expected):
            for actual, wanted in zip(actual_row, expected_row):
                self.assertAlmostEqual(actual, wanted, places=12)
        self.assertAlmostEqual(kin.SUCTION_FACE_CLOCKING_RAD, math.pi / 2, places=12)

    def test_tool_contact_calibration_changes_flange_not_part_geometry(self):
        part = {
            "id": "p", "label": "Box", "type": "box", "graspable": True,
            "position": {"x": 0.18, "y": 0.03, "z": 0.025},
            "size": {"x": 0.06, "y": 0.04, "z": 0.05}, "orientationDeg": 0,
        }
        original_position = dict(part["position"])
        original_size = dict(part["size"])
        with tempfile.TemporaryDirectory() as directory:
            cell = Workcell(Path(directory))
            before = cell._tcp_to_flange_point((0.18, 0.03, 0.03), [180, 0, 0])
            cell.set_coordinate_planner_config({
                "observedContactMissMm": {"left": 8, "forward": 0, "high": 12}
            })
            after = cell._tcp_to_flange_point((0.18, 0.03, 0.03), [180, 0, 0])
        self.assertGreater(math.dist(before, after), 0.01)
        self.assertEqual(part["position"], original_position)
        self.assertEqual(part["size"], original_size)

    def test_object_local_pickup_offsets_rotate_with_part_yaw(self):
        part = {"orientationDeg": 90}
        world = Workcell._object_local_offset_world(part, {"x": 0.01, "y": 0, "z": -0.002})
        self.assertAlmostEqual(world[0], 0, places=9)
        self.assertAlmostEqual(world[1], 0.01, places=9)
        self.assertAlmostEqual(world[2], -0.002, places=9)

    def test_suction_uses_top_contact_preload_and_rejects_cup_overhang(self):
        cell = self._tool_cell("suction_gripper")
        part = {
            "id": "box", "label": "Box", "type": "box", "graspable": True,
            "position": {"x": 0.18, "y": 0.03, "z": 0.025},
            "size": {"x": 0.06, "y": 0.05, "z": 0.05}, "orientationDeg": 0,
            "pickupProfiles": {"suction_gripper": {
                "offsetLocalM": {"x": 0, "y": 0, "z": 0}, "contactPreloadM": 0.002,
            }},
        }
        grasp = cell._suction_grasp(part)
        self.assertTrue(grasp["ok"])
        self.assertAlmostEqual(grasp["graspPoint"][2], 0.048, places=9)
        part["pickupProfiles"]["suction_gripper"]["offsetLocalM"]["x"] = 0.025
        rejected = cell._suction_grasp(part)
        self.assertFalse(rejected["ok"])
        self.assertIn("complete", rejected["error"])

    def test_pump_v2_and_named_legacy_output_sequences(self):
        on, off = MyCobotDriver._suction_profile_sequences("pump_v2")
        self.assertEqual(on, "2:1,5:0")
        self.assertEqual(off, "5:1,2:0,sleep:1.0,2:1")
        legacy = MyCobotDriver._suction_profile_sequences("legacy_split_valve")
        self.assertNotEqual((on, off), legacy)

    def test_suction_plan_holds_one_orientation_and_avoids_side_pinch_height(self):
        cell = self._tool_cell("suction_gripper")
        part = {
            "id": "box", "label": "Box", "type": "box", "graspable": True,
            "position": {"x": 0.18, "y": 0.03, "z": 0.025},
            "size": {"x": 0.06, "y": 0.05, "z": 0.05}, "orientationDeg": 37,
        }
        plan = cell._plan_single_pick_coordinate(
            part,
            {"kind": "point", "position": {"x": 0.13, "y": -0.04, "z": 0}},
            2, None, "missing",
        )
        self.assertTrue(plan["ok"])
        motion = {step["name"]: step for step in plan["steps"] if step.get("coordsMm")}
        orientation = motion["approach"]["coordsMm"][3:]
        self.assertEqual(motion["descend"]["coordsMm"][3:], orientation)
        self.assertEqual(motion["lift"]["coordsMm"][3:], orientation)
        self.assertEqual(motion["lower"]["coordsMm"][3:], orientation)
        self.assertEqual(motion["retreat"]["coordsMm"][3:], orientation)
        self.assertAlmostEqual(motion["descend"]["targetTcpPoseM"]["z"], 0.048, places=6)
        self.assertEqual(motion["descend"]["grasp"]["strategy"], "suction_top_surface")

    def test_cross_table_suction_carry_is_subdivided_without_moving_drop_target(self):
        """Regression for the July 21 object-to-bin J1 discontinuity."""
        cell = self._tool_cell("suction_gripper")
        part = {
            "id": "part-23", "label": "Part 3", "type": "box", "graspable": True,
            "position": {"x": 0.18863, "y": -0.15078, "z": 0.0254},
            "size": {"x": 0.0508, "y": 0.0254, "z": 0.0508},
            "orientationDeg": -82.3,
        }
        bin_obj = {
            "id": "bin-7", "label": "Bin A",
            "position": {"x": 0.16430, "y": 0.23183, "z": 0.0},
            "outer": {"x": 0.125, "y": 0.125, "z": 0.07},
            "wallThickness": 0.006, "orientationDeg": 0.0,
        }
        segment = cell._plan_single_pick_coordinate(
            part, {"kind": "bin", "bin": bin_obj}, 2, None,
            "canonical_top_down_suction",
        )
        self.assertTrue(segment["ok"])
        transfer_steps = [step for step in segment["steps"] if step.get("name") == "transfer"]
        self.assertEqual(len(transfer_steps), 2)
        carry = next(step for step in segment["steps"] if step.get("name") == "carry")
        expected_drop = Workcell.reachable_bin_drop_xy(
            bin_obj, Workcell.bin_geometry(bin_obj), part["size"]
        )
        self.assertAlmostEqual(carry["targetTcpPoseM"]["x"], expected_drop["x"], places=9)
        self.assertAlmostEqual(carry["targetTcpPoseM"]["y"], expected_drop["y"], places=9)
        wall_top = Workcell.bin_geometry(bin_obj)["wallTopZ"]
        object_bottom_during_carry = (
            carry["targetTcpPoseM"]["z"] - (part["size"]["z"] - 0.002)
        )
        self.assertGreaterEqual(object_bottom_during_carry, wall_top + 0.02 - 1e-9)

        plan = {
            "ok": True, "mode": "coordinate_program", "physicalReady": True,
            "steps": [{"stateId": "seq01_home", "name": "home", "robotCommand": "home"}]
            + segment["steps"],
        }
        service = RobotService(None, 115200, 0.1)
        service.set_end_effector("suction_gripper")
        service.set_tool_profile(cell.coordinate_planner["toolProfiles"]["suction_gripper"])
        service.add_coordinate_preview(plan, HOME_ANGLES)
        preview = plan["coordinatePreview"]
        self.assertTrue(preview["ok"], preview)
        place_states = [state for state in preview["states"] if "_s5_" in state["stateId"]]
        self.assertTrue(place_states)
        self.assertLessEqual(max(state["maxJointStepDeg"] for state in place_states), 75.0)
        self.assertTrue(all(
            state.get("suctionJ6LockErrorDeg", 0.0) <= 0.5
            for state in preview["states"]
        ))
        self.assertTrue(all(
            abs(float(step["previewAngles"][5]) - HOME_ANGLES[5]) <= 0.5
            for step in segment["steps"]
            if step.get("coordsMm")
        ))

    def test_low_pick_stages_above_object_and_cross_base_transfer_uses_clearance_arc(self):
        cell = self._tool_cell("suction_gripper")
        part = {
            "id": "part-23", "label": "Part 3", "type": "box", "graspable": True,
            "position": {"x": 0.1269, "y": 0.1819, "z": 0.0225},
            "size": {"x": 0.0508, "y": 0.0254, "z": 0.045},
            "orientationDeg": -22.8,
        }
        bin_obj = {
            "id": "bin-7", "label": "Bin A",
            "position": {"x": 0.0242, "y": -0.3048, "z": 0.0},
            "outer": {"x": 0.125, "y": 0.125, "z": 0.07},
            "wallThickness": 0.006, "orientationDeg": 0.0,
        }
        segment = cell._plan_single_pick_coordinate(
            part, {"kind": "bin", "bin": bin_obj}, 2, None,
            "canonical_top_down_suction", route_low_approach=True,
        )
        staging = next(step for step in segment["steps"] if step.get("name") == "approach_staging")
        approach = next(step for step in segment["steps"] if step.get("name") == "approach")
        self.assertEqual(staging["targetTcpPoseM"]["x"], approach["targetTcpPoseM"]["x"])
        self.assertEqual(staging["targetTcpPoseM"]["y"], approach["targetTcpPoseM"]["y"])
        self.assertGreater(staging["targetTcpPoseM"]["z"], approach["targetTcpPoseM"]["z"])
        self.assertEqual(approach["coordMode"], 1)

        transfer = [step for step in segment["steps"] if step.get("name") == "transfer"]
        self.assertEqual(len(transfer), 2)
        self.assertTrue(all(
            math.hypot(step["targetTcpPoseM"]["x"], step["targetTcpPoseM"]["y"]) >= 0.13 - 1e-9
            for step in transfer
        ))

    def test_outer_suction_pick_omits_unreachable_high_staging_regression(self):
        """A reachable low pick must not inherit an unreachable 160 mm staging pose."""
        cell = self._tool_cell("suction_gripper")
        part = {
            "id": "part-23", "label": "Part 3", "type": "box", "graspable": True,
            "position": {"x": 0.26332, "y": 0.07758, "z": 0.019939},
            "size": {"x": 0.0508, "y": 0.0254, "z": 0.039878},
            "orientationDeg": 4.36,
        }
        segment = cell._plan_single_pick_coordinate(
            part,
            {"kind": "position", "position": {"x": 0.2633, "y": -0.0813, "z": 0.0}},
            2, None, "canonical_top_down_suction", route_low_approach=True,
        )
        self.assertTrue(segment["ok"], segment)
        self.assertNotIn("approach_staging", [step.get("name") for step in segment["steps"]])
        approach = next(step for step in segment["steps"] if step.get("name") == "approach")
        self.assertEqual(approach["coordMode"], 0)
        self.assertLess(approach["targetTcpPoseM"]["z"], 0.10)
        self.assertTrue(any("high staging point omitted" in note for note in segment["notes"]))

        service = RobotService(None, 115200, 0.1)
        service.set_end_effector("suction_gripper")
        service.set_tool_profile(cell.coordinate_planner["toolProfiles"]["suction_gripper"])
        coordinate_steps = [step for step in segment["steps"] if step.get("coordsMm")]
        preview = service._preview_coordinate_group(
            HostKinematicsPreviewRobot(
                "suction_gripper", [0.0, 0.0, 0.0], 0.072
            ),
            coordinate_steps,
            [1.05, -0.43, -0.52, -0.70, 1.05, -44.38],
        )
        self.assertTrue(preview["ok"], preview)
        self.assertTrue(all(
            state.get("suctionJ6LockErrorDeg", 0.0) <= 0.5
            for state in preview["states"]
        ))

    def test_tool_contact_calibration_uses_pick_yaw_for_local_correction(self):
        with tempfile.TemporaryDirectory() as directory:
            cell = Workcell(Path(directory))
            cell.set_coordinate_planner_config({
                "calibrationJawYawDeg": 90,
                "observedContactMissMm": {"left": 10, "forward": 0, "high": 0},
            })
            correction = cell.coordinate_planner["toolProfiles"]["adaptive_gripper"]["tcpCorrectionLocalM"]
        self.assertGreater(math.dist((0, 0, 0), tuple(correction.values())), 0.009)
        self.assertLess(math.dist((0, 0, 0), tuple(correction.values())), 0.011)

    def test_json_safe_breaks_response_cycles(self):
        payload = {"ok": True}
        payload["config"] = payload
        cleaned = json_safe(payload)
        self.assertTrue(cleaned["ok"])
        self.assertIsNone(cleaned["config"])

    def test_flange_tcp_transform_is_explicit_and_composable(self):
        angles = [12.0, -25.0, 44.0, 18.0, -31.0, -45.0]
        flange = kin.forward_flange_kinematics(angles)
        self.assertEqual(kin.forward_kinematics(angles), kin.tcp_from_flange(*flange))
        self.assertGreater(math.dist(flange[0], kin.forward_kinematics(angles)[0]), 0.05)
        recovered = kin.flange_from_tcp(*kin.tcp_from_flange(*flange))
        self.assertLess(math.dist(flange[0], recovered[0]), 1e-12)
        self.assertLess(kin.pose_residual((0, 0, 0), flange[1], (0, 0, 0), recovered[1])[1], 1e-12)

    def test_canonical_top_down_pose_centers_jaws_without_tilt(self):
        target = (0.22775, -0.05583, 0.038)
        for yaw in (0.0, 45.0, 90.0, -90.0, 179.8):
            flange = kin.top_down_flange_pose(target, yaw)
            tcp = kin.tcp_from_flange(*flange)
            diagnostics = kin.tool_axis_diagnostics(flange[1])
            self.assertLess(math.dist(target, tcp[0]), 1e-12)
            self.assertLess(diagnostics["approachTiltDeg"], 1e-9)
            self.assertTrue(diagnostics["topDown"])
            self.assertAlmostEqual(abs(((diagnostics["jawYawDeg"] - yaw + 180) % 360) - 180), 0.0, places=8)

    def test_adaptive_gripper_tcp_includes_cad_jaw_lateral_offset(self):
        self.assertEqual(kin._TOOL_POCKET, (0.0, 0.078, 0.004))
        flange = kin.top_down_flange_pose((0.23, -0.06, 0.04), -84.8)
        tcp = kin.tcp_from_flange(*flange)
        self.assertLess(math.dist(tcp[0], (0.23, -0.06, 0.04)), 1e-12)

    def test_rpy_matrix_round_trip(self):
        for rpy in ([0, 0, 0], [179.7, 1.3, -44.6], [30, -20, 120]):
            rotation = kin.rotation_from_rpy_deg(rpy)
            recovered = kin.rpy_deg_from_rotation(rotation)
            residual = kin.pose_residual((0, 0, 0), rotation, (0, 0, 0), kin.rotation_from_rpy_deg(recovered))
            self.assertLess(residual[1], 1e-9)

    def test_fk_ik_reachable_property(self):
        random.seed(280)
        limits = [kin.JOINT_LIMITS_DEG[i] for i in range(1, 7)]
        for _ in range(100):
            expected = [random.uniform(low * 0.65, high * 0.65) for low, high in limits]
            position, rotation = kin.forward_kinematics(expected)
            seed = [value + random.uniform(-8.0, 8.0) for value in expected]
            solved = kin.solve_pose(position, rotation, seed)
            self.assertIsNotNone(solved)
            actual_position, actual_rotation = kin.forward_kinematics(solved)
            position_error, orientation_error = kin.pose_residual(
                actual_position, actual_rotation, position, rotation
            )
            self.assertLessEqual(position_error, kin.IK_POSITION_TOLERANCE_M)
            self.assertLessEqual(orientation_error, kin.IK_ORIENTATION_TOLERANCE_RAD)

    def test_unreachable_top_down_is_rejected(self):
        self.assertIsNone(kin.solve_top_down((0.5, 0.0, 0.1)))

    def test_coordinate_bounds_reject_bad_shape_and_range(self):
        self.assertTrue(validate_coordinate_bounds([1, 2], "bad"))
        self.assertTrue(validate_coordinate_bounds([999, 0, 100, 0, 0, 0], "bad"))
        self.assertFalse(validate_coordinate_bounds([100, 0, 100, 180, 0, -180], "ok"))

    def test_generated_coordinate_margin_only_defers_small_xy_overrun_to_ik(self):
        regression = [290.5, 3.68, 188.0, 180.0, 0.0, -44.88]
        self.assertTrue(validate_coordinate_bounds(regression, "strict"))
        self.assertFalse(validate_coordinate_bounds(
            regression, "generated", xy_margin_mm=15.0
        ))
        self.assertTrue(validate_coordinate_bounds(
            [300.0, 0.0, 188.0, 180.0, 0.0, 0.0],
            "still_outside",
            xy_margin_mm=15.0,
        ))

    def test_plan_validation_uses_margin_only_for_generated_tcp_steps(self):
        coords = [290.5, 3.68, 188.0, 180.0, 0.0, -44.88]
        generated = [{
            "stateId": "generated_pick",
            "coordsMm": coords,
            "coordMode": 0,
            "targetTcpPoseM": {"x": 0.2905, "y": 0.00368, "z": 0.188},
        }]
        taught = [{"stateId": "taught_move", "coordsMm": coords, "coordMode": 0}]
        self.assertIsNone(RobotService.validate_plan_steps(generated))
        self.assertIn("invalid coord value on x", RobotService.validate_plan_steps(taught))

    def test_tcp_to_flange_uses_modeled_adaptive_gripper_transform(self):
        cell = Workcell.__new__(Workcell)
        cell.end_effector = "adaptive_gripper"
        cell.coordinate_planner = {"pickHeightBiasM": 0.0}
        target_tcp = (0.1, 0.2, 0.03)
        rpy = [180.0, 0.0, 0.0]
        flange = cell._tcp_to_flange_point(target_tcp, rpy)
        flange_rotation = kin.rotation_from_rpy_deg(rpy)
        recovered_tcp = kin.tcp_from_flange(flange, flange_rotation)[0]
        self.assertLess(math.dist(recovered_tcp, target_tcp), 1e-9)

    def test_top_down_side_pinch_is_centered_on_rotated_objects(self):
        cell = Workcell.__new__(Workcell)
        cell.end_effector = "adaptive_gripper"
        cell.coordinate_planner = {"pickHeightBiasM": 0.0}
        for index, yaw in enumerate((0.0, 37.0, 90.0, -174.8)):
            part = {
                "id": f"part-{index}", "label": f"Part {index}", "type": "box", "graspable": True,
                "position": {"x": 0.16 + index * 0.01, "y": -0.08 + index * 0.03, "z": 0.022},
                "size": {"x": 0.0508, "y": 0.0254, "z": 0.044},
                "orientationDeg": yaw,
            }
            candidates = cell.surface_grasp_candidates(part)
            self.assertTrue(candidates)
            for candidate in candidates:
                self.assertAlmostEqual(candidate["graspPoint"][0], part["position"]["x"], places=9)
                self.assertAlmostEqual(candidate["graspPoint"][1], part["position"]["y"], places=9)
                self.assertEqual(candidate["surfaceGripDepthM"], 0.0)

    def test_grasp_depth_centers_contact_segment_without_unexplained_height_ratio(self):
        cell = Workcell.__new__(Workcell)
        cell.end_effector = "adaptive_gripper"
        cell.coordinate_planner = {"pickHeightBiasM": 0.0}
        fixtures = (("box", 0.020), ("rectangle", 0.044), ("open-box", 0.100))
        for index, (part_type, height) in enumerate(fixtures):
            part = {
                "id": f"depth-{index}", "label": f"Depth {index}", "type": part_type, "graspable": True,
                "position": {"x": 0.20, "y": -0.04, "z": height / 2.0},
                "size": {"x": 0.050, "y": 0.025, "z": height}, "orientationDeg": 31.0,
            }
            candidate = cell.surface_grasp_candidates(part)[0]
            model = candidate["graspHeightModel"]
            expected_jaw_z = max(height / 2.0, cell.minimum_adaptive_gripper_jaw_z())
            self.assertAlmostEqual(candidate["graspPoint"][2], expected_jaw_z, places=9)
            self.assertAlmostEqual(model["jawCenterTargetZ"], expected_jaw_z, places=9)
            self.assertAlmostEqual(
                model["fingertipLowTargetZ"], expected_jaw_z - model["fingerContactLengthM"], places=9
            )
            self.assertGreaterEqual(model["tableClearanceM"], model["minimumTableClearanceM"])
            self.assertGreater(model["actualFingerOverlapM"], 0.0)
            self.assertGreaterEqual(model["actualFingerOverlapM"] + 1e-9, model["desiredFingerOverlapM"])

    def test_pick_height_bias_is_bounded_and_approach_clears_object_top(self):
        cell = Workcell.__new__(Workcell)
        cell.end_effector = "adaptive_gripper"
        part = {
            "id": "bias-depth", "label": "Bias Depth", "type": "box", "graspable": True,
            "position": {"x": 0.21, "y": 0.05, "z": 0.05},
            "size": {"x": 0.05, "y": 0.03, "z": 0.10}, "orientationDeg": 0.0,
        }
        for requested, applied in ((-1.0, -0.008), (1.0, 0.008)):
            cell.coordinate_planner = {"pickHeightBiasM": requested}
            candidate = cell.surface_grasp_candidates(part)[0]
            model = candidate["graspHeightModel"]
            self.assertAlmostEqual(model["pickHeightBiasM"], applied, places=9)
            self.assertGreaterEqual(
                candidate["pregraspPoint"][2], model["objectTopZ"] + 0.04 - 1e-9
            )

    def test_minimum_table_clearance_is_configurable_and_bounded(self):
        cell = Workcell.__new__(Workcell)
        cell.end_effector = "adaptive_gripper"
        part = {
            "id": "short", "label": "Short", "type": "box", "graspable": True,
            "position": {"x": 0.2, "y": 0.0, "z": 0.01},
            "size": {"x": 0.04, "y": 0.03, "z": 0.02}, "orientationDeg": 0.0,
        }
        for requested, expected in ((-1.0, 0.002), (0.007, 0.007), (1.0, 0.012)):
            cell.coordinate_planner = {"pickHeightBiasM": 0.0, "minimumTableClearanceM": requested}
            model = cell.surface_grasp_candidates(part)[0]["graspHeightModel"]
            self.assertAlmostEqual(model["minimumTableClearanceM"], expected, places=9)
            self.assertAlmostEqual(model["tableClearanceM"], expected, places=9)

    def test_pick_depth_diagnostics_propagate_to_firmware_target(self):
        cell = Workcell.__new__(Workcell)
        cell.end_effector = "adaptive_gripper"
        cell.coordinate_planner = {"pickHeightBiasM": 0.0}
        part = {
            "id": "tagged-depth", "label": "Tagged Depth", "type": "box", "graspable": True,
            "position": {"x": 0.225, "y": -0.055, "z": 0.0254},
            "size": {"x": 0.0508, "y": 0.0254, "z": 0.0508}, "orientationDeg": -83.8,
        }
        segment = cell._plan_single_pick_coordinate(
            part, {"kind": "point", "position": {"x": 0.18, "y": 0.12, "z": 0.0}},
            2, None, "canonical_top_down",
        )
        self.assertTrue(segment["ok"])
        descend, grip = segment["steps"][1:3]
        diagnostics = descend["grasp"]["heightModel"]
        required = {
            "objectBottomZ", "objectCenterZ", "objectTopZ", "desiredFingerOverlapM",
            "actualFingerOverlapM", "jawCenterTargetZ", "fingertipLowTargetZ",
            "tableClearanceM", "minimumTableClearanceM", "pickHeightBiasM",
        }
        self.assertTrue(required.issubset(diagnostics))
        self.assertEqual(grip["grasp"]["heightModel"], diagnostics)
        self.assertAlmostEqual(descend["targetTcpPoseM"]["z"], diagnostics["jawCenterTargetZ"], places=4)
        flange_position = tuple(value / 1000.0 for value in descend["coordsMm"][:3])
        flange_rotation = kin.rotation_from_rpy_deg(descend["coordsMm"][3:6])
        recovered_tcp = kin.tcp_from_flange(flange_position, flange_rotation)[0]
        target_tcp = tuple(descend["targetTcpPoseM"][axis] for axis in ("x", "y", "z"))
        self.assertLess(math.dist(recovered_tcp, target_tcp), 1e-4)

    def test_coordinate_pick_approach_and_descend_use_object_center_xy(self):
        cell = Workcell.__new__(Workcell)
        cell.end_effector = "adaptive_gripper"
        cell.coordinate_planner = {"pickHeightBiasM": 0.0}
        part = {
            "id": "center-test", "label": "Center Test", "type": "box", "graspable": True,
            "position": {"x": 0.22775, "y": -0.05583, "z": 0.022},
            "size": {"x": 0.0508, "y": 0.0254, "z": 0.044},
            "orientationDeg": -174.8,
        }
        segment = cell._plan_single_pick_coordinate(
            part,
            {"kind": "point", "position": {"x": 0.18, "y": 0.12, "z": 0.0}},
            1,
            [179.7, 1.4, -44.6],
            "captured",
        )
        self.assertTrue(segment["ok"])
        for step in segment["steps"][:2]:
            self.assertAlmostEqual(step["targetTcpPoseM"]["x"], part["position"]["x"], places=4)
            self.assertAlmostEqual(step["targetTcpPoseM"]["y"], part["position"]["y"], places=4)
            flange_position = tuple(value / 1000.0 for value in step["coordsMm"][:3])
            flange_rotation = kin.rotation_from_rpy_deg(step["coordsMm"][3:6])
            recovered_tcp = kin.tcp_from_flange(flange_position, flange_rotation)[0]
            self.assertAlmostEqual(recovered_tcp[0], part["position"]["x"], places=4)
            self.assertAlmostEqual(recovered_tcp[1], part["position"]["y"], places=4)

    def test_reachable_top_down_pick_matrix_is_centered_and_complete(self):
        cell = Workcell.__new__(Workcell)
        cell.end_effector = "adaptive_gripper"
        cell.coordinate_planner = {"pickHeightBiasM": 0.0}
        cell.end_effectors = {}
        service = RobotService(None, 115200, 0.1)
        host = HostKinematicsPreviewRobot()
        cases = (
            ("center", "box", 0.23, 0.00, 0.0, 0.040, 0.040, 0.044),
            ("front", "rectangle", 0.25, 0.00, 0.0, 0.050, 0.025, 0.044),
            ("rear", "open-box", 0.215, 0.00, 0.0, 0.040, 0.030, 0.044),
            ("left", "box", 0.23, 0.10, 0.0, 0.060, 0.035, 0.050),
            ("right", "rectangle", 0.23, -0.10, 0.0, 0.045, 0.020, 0.060),
            ("yaw45", "box", 0.21, 0.10, 45.0, 0.0508, 0.0254, 0.044),
            ("yaw90", "rectangle", 0.21, 0.15, 90.0, 0.0508, 0.0254, 0.044),
            ("yaw-90", "open-box", 0.21, 0.10, -90.0, 0.0508, 0.0254, 0.044),
            ("yaw179", "box", 0.23, 0.00, 179.0, 0.0508, 0.0254, 0.044),
            # Purple screenshot regression.
            ("purple-near-base", "box", 0.22775, -0.05583, -174.8, 0.0508, 0.0254, 0.044),
        )
        for name, part_type, x, y, yaw, sx, sy, sz in cases:
            with self.subTest(name=name):
                part = {
                    "id": name, "label": name, "type": part_type, "graspable": True,
                    "position": {"x": x, "y": y, "z": sz / 2.0},
                    "size": {"x": sx, "y": sy, "z": sz},
                    "orientationDeg": yaw,
                }
                segment = cell._plan_single_pick_coordinate(
                    part,
                    {"kind": "point", "position": {"x": 0.22, "y": 0.10, "z": 0.0}},
                    1,
                    None,
                    "canonical_top_down",
                )
                self.assertTrue(segment["ok"])
                coordinate_steps = [step for step in segment["steps"] if step.get("coordsMm")]
                preview = service._preview_coordinate_group(
                    host, coordinate_steps, [0, 0, 0, 0, 0, -45]
                )
                self.assertTrue(preview["ok"], preview.get("error"))
                self.assertEqual(len(preview["states"]), len(coordinate_steps))
                for state in preview["states"]:
                    self.assertLessEqual(state["plannedJawCenterErrorMm"], 1.0)
                    self.assertLessEqual(state["jawCenterErrorMm"], 1.0)
                    self.assertLessEqual(state["toolApproachTiltDeg"], 3.0)
                pick_steps = coordinate_steps[:3]
                self.assertTrue(all(abs(step["targetTcpPoseM"]["x"] - x) <= 0.001 for step in pick_steps))
                self.assertTrue(all(abs(step["targetTcpPoseM"]["y"] - y) <= 0.001 for step in pick_steps))
                pick_rpy = [tuple(step["coordsMm"][3:6]) for step in pick_steps]
                self.assertEqual(len(set(pick_rpy)), 1)
                height_model = segment["steps"][1]["grasp"]["heightModel"]
                self.assertGreaterEqual(
                    segment["steps"][0]["targetTcpPoseM"]["z"],
                    height_model["objectTopZ"] + 0.04 - 1e-9,
                )


class PreviewValidationTests(unittest.TestCase):
    class FixedRobot:
        def __init__(self, angles):
            self.angles = angles

        def solve_inv_kinematics(self, coords, current):
            return list(self.angles)

    def test_home_only_plan_is_physically_ready_without_coordinate_ik_states(self):
        with tempfile.TemporaryDirectory() as directory:
            cell = Workcell(Path(directory))
            plan = cell.plan_program([{"type": "home"}], HOME_ANGLES, "voice home")
        self.assertTrue(plan["ok"], plan)
        self.assertEqual(plan["mode"], "coordinate_program")
        service = RobotService(None, 115200, 0.1)
        service.add_coordinate_preview(plan, HOME_ANGLES)
        self.assertTrue(plan["coordinatePreview"]["ok"], plan["coordinatePreview"])
        self.assertEqual(plan["coordinatePreview"]["requiredStates"], 0)
        self.assertTrue(plan["coordinatePreview"]["jointOnlyPlan"])
        self.assertTrue(plan["physicalReady"])

    def test_complete_group_selects_constant_orientation(self):
        steps = []
        for state_id, tcp_position, coord_mode in (
            ("seq01_s1_approach", (0.18, 0.02, 0.16), 0),
            ("seq01_s2_descend", (0.18, 0.02, 0.05), 1),
        ):
            flange_position, flange_rotation = kin.top_down_flange_pose(tcp_position, 90.0)
            rpy = kin.rpy_deg_from_rotation(flange_rotation)
            steps.append({
                "stateId": state_id,
                "coordsMm": [value * 1000.0 for value in flange_position] + list(rpy),
                "targetTcpPoseM": {"x": tcp_position[0], "y": tcp_position[1], "z": tcp_position[2]},
                "desiredJawYawDeg": 90.0,
                "coordMode": coord_mode,
            })
        host = HostKinematicsPreviewRobot()
        start = host.solve_inv_kinematics(steps[0]["coordsMm"], [0, 0, 0, 0, 0, -45])
        result = RobotService(None, 115200, 0.1)._preview_coordinate_group(host, steps, start)
        self.assertTrue(result["ok"])
        self.assertEqual(steps[0]["coordsMm"][3:6], steps[1]["coordsMm"][3:6])
        self.assertLessEqual(abs(steps[0]["selectedOrientation"]["toolApproachTiltDeg"]), 3.0)

    def test_repeated_preview_does_not_compound_orientation_offsets(self):
        tcp_position = (0.18, 0.02, 0.10)
        position, rotation = kin.top_down_flange_pose(tcp_position, -90.0)
        base_rpy = list(kin.rpy_deg_from_rotation(rotation))
        coords = [value * 1000.0 for value in position] + base_rpy
        steps = [{
            "stateId": "seq01_s1_approach",
            "coordsMm": list(coords),
            "baseToolRpyDeg": list(base_rpy),
            "targetTcpPoseM": {"x": tcp_position[0], "y": tcp_position[1], "z": tcp_position[2]},
            "desiredJawYawDeg": -90.0,
        }]
        service = RobotService(None, 115200, 0.1)
        host = HostKinematicsPreviewRobot()
        start = host.solve_inv_kinematics(steps[0]["coordsMm"], [0, 0, 0, 0, 0, -45])
        first = service._preview_coordinate_group(host, steps, start)
        first_rpy = list(steps[0]["coordsMm"][3:6])
        second = service._preview_coordinate_group(host, steps, start)
        self.assertTrue(first["ok"] and second["ok"])
        self.assertEqual(first_rpy, steps[0]["coordsMm"][3:6])

    def test_equivalent_jaw_yaw_recomputes_complete_flange_pose(self):
        tcp_position = (0.21, 0.15, 0.078)
        original_flange, original_rotation = kin.top_down_flange_pose(tcp_position, -180.0)
        original_rpy = list(kin.rpy_deg_from_rotation(original_rotation))
        step = {
            "stateId": "seq01_s1_approach",
            "coordsMm": [value * 1000.0 for value in original_flange] + original_rpy,
            "baseToolRpyDeg": original_rpy,
            "targetTcpPoseM": {"x": tcp_position[0], "y": tcp_position[1], "z": tcp_position[2]},
            "targetFlangePoseM": {"x": original_flange[0], "y": original_flange[1], "z": original_flange[2]},
            "desiredJawYawDeg": -180.0,
            "coordMode": 0,
        }
        result = RobotService(None, 115200, 0.1)._preview_coordinate_group(
            HostKinematicsPreviewRobot(), [step], [0, 0, 0, 0, 0, -45]
        )
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(step["selectedOrientation"]["yawOffsetDeg"], 180.0)
        self.assertGreater(math.dist(original_flange[:2], tuple(value / 1000.0 for value in step["coordsMm"][:2])), 0.01)
        selected_position = tuple(value / 1000.0 for value in step["coordsMm"][:3])
        selected_rotation = kin.rotation_from_rpy_deg(step["coordsMm"][3:6])
        actual_tcp = kin.tcp_from_flange(selected_position, selected_rotation)[0]
        self.assertLess(math.dist(actual_tcp, tcp_position), 1e-5)
        self.assertEqual(
            step["targetFlangePoseM"],
            {"x": round(selected_position[0], 6), "y": round(selected_position[1], 6), "z": round(selected_position[2], 6)},
        )

    def test_invalid_and_discontinuous_results_are_rejected(self):
        invalid = RobotService._validate_firmware_ik([0, 0, 0, 0, 0, 0], [math.nan] * 6, [0] * 6)
        self.assertFalse(invalid["ok"])
        jump = RobotService._joint_solution_diagnostics([100, 0, 0, 0, 0, 0], [0] * 6)
        self.assertFalse(jump["ok"])
        self.assertIn("joint_discontinuity", jump["rejectionReasons"])
        repeated = RobotService._joint_solution_diagnostics([82.41] * 6, [0] * 6)
        self.assertFalse(repeated["ok"])
        self.assertIn("firmware_repeated_angle_failure_pattern", repeated["rejectionReasons"])

    def test_approximate_firmware_roundtrip_is_a_warning_within_physical_envelope(self):
        angles = [0.0, -45.0, 45.0, 0.0, 0.0, -45.0]
        position, rotation = kin.firmware_flange_kinematics(angles)
        target = [position[0] * 1000.0, position[1] * 1000.0, position[2] * 1000.0,
                  *kin.rpy_deg_from_rotation(rotation)]
        firmware_fk = list(target)
        firmware_fk[0] -= 7.0
        result = RobotService._validate_firmware_ik(target, angles, [0, 0, 0, 0, 0, -45], firmware_fk)
        self.assertTrue(result["ok"], result.get("rejectionReasons"))
        self.assertIn("firmware_fk_roundtrip_above_precision_target", result["accuracyWarnings"])

    def test_firmware_self_roundtrip_cannot_hide_host_unreachable_pose(self):
        target = [268.99, -5.239, 196.6, -180.0, 0.0, -38.808]
        angles = [12.52, -60.13, 0.0, -29.86, 0.0, -38.67]
        result = RobotService._validate_firmware_ik(
            target, angles, [0, 0, 0, 0, 0, -45], list(target)
        )
        self.assertFalse(result["ok"])
        self.assertFalse(result["hostIkReachable"])
        self.assertIn("host_ik_unreachable", result["rejectionReasons"])

    def test_recorded_refused_target_is_rejected_before_motion(self):
        class FalsePositiveFirmware:
            def __init__(self):
                self.last_coords = None
                self.motion_commands = 0
                self.ik_calls = 0

            def solve_inv_kinematics(self, coords, current):
                self.ik_calls += 1
                self.last_coords = list(coords)
                return [12.52, -60.13, 0.0, -29.86, 0.0, -38.67]

            def angles_to_coords(self, angles):
                return list(self.last_coords)

            def send_coords(self, coords, speed, mode):
                self.motion_commands += 1

        target = [267.56, 3.68, 203.8, -180.0, 0.0, -44.88]
        flange_position = tuple(value / 1000.0 for value in target[:3])
        flange_rotation = kin.rotation_from_rpy_deg(target[3:6])
        tcp_position, _ = kin.tcp_from_flange(flange_position, flange_rotation)
        axes = kin.tool_axis_diagnostics(flange_rotation)
        step = {
            "stateId": "seq02_s1_approach",
            "coordsMm": target,
            "baseToolRpyDeg": target[3:6],
            "targetTcpPoseM": dict(zip(("x", "y", "z"), tcp_position)),
            "desiredJawYawDeg": axes["jawYawDeg"],
            "coordMode": 0,
        }
        robot = FalsePositiveFirmware()
        result = RobotService(None, 115200, 0.1)._preview_coordinate_group(
            robot, [step], [0.0, 0.0, 0.0, 0.0, 0.0, -45.0]
        )
        self.assertFalse(result["ok"])
        self.assertEqual(robot.motion_commands, 0)
        self.assertEqual(
            robot.ik_calls,
            result["planningDiagnostics"]["firmwareIkCalls"],
        )
        self.assertEqual(step["coordsMm"], target)
        self.assertEqual(result["suggestedInwardShiftMm"], 5.0)
        self.assertTrue(any(
            "host_ik_unreachable" in state.get("rejectionReasons", [])
            for candidate in result.get("rejectedCandidates", [])
            for state in candidate.get("states", [])
        ))

    def test_unreachable_raised_suction_target_finishes_bounded_search(self):
        target_tcp = (0.291, 0.013, 0.228)
        flange_position, flange_rotation = kin.top_down_flange_pose(
            target_tcp, 0.0, "suction_gripper", [0.0, 0.0, 0.0], 0.072
        )
        rpy = list(kin.rpy_deg_from_rotation(flange_rotation))
        step = {
            "stateId": "raised_surface_suction_regression",
            "coordsMm": [value * 1000.0 for value in flange_position] + rpy,
            "baseToolRpyDeg": rpy,
            "targetTcpPoseM": dict(zip(("x", "y", "z"), target_tcp)),
            "activeTool": "suction_gripper",
            "toolProfile": {
                "tcpCorrectionLocalM": {"x": 0.0, "y": 0.0, "z": 0.0},
                "geometry": {"flangeToContactM": 0.072},
            },
            "coordMode": 0,
        }
        started = time.perf_counter()
        result = RobotService(None, 115200, 0.1)._preview_coordinate_group(
            HostKinematicsPreviewRobot(), [step], [0.0, 0.0, 0.0, 0.0, 0.0, -45.0]
        )
        elapsed = time.perf_counter() - started
        diagnostics = result.get("planningDiagnostics") or {}

        self.assertFalse(result["ok"])
        self.assertLess(elapsed, 8.0, f"bounded search took {elapsed:.2f}s")
        self.assertLessEqual(diagnostics.get("exhaustiveFallbackCandidates", 99), 2)
        self.assertLessEqual(diagnostics.get("exhaustiveHostSolves", 99), 2)
        self.assertEqual(diagnostics.get("firmwareIkCalls"), 0)

    def test_fast_host_ik_failure_reports_rankable_residuals(self):
        diagnostics = {}
        solution = kin.solve_pose(
            (0.45, 0.0, 0.45),
            kin.rotation_from_rpy_deg([-180.0, 0.0, 0.0]),
            [0.0, 0.0, 0.0, 0.0, 0.0, -45.0],
            exhaustive=False,
            diagnostics=diagnostics,
        )
        self.assertIsNone(solution)
        self.assertEqual(diagnostics["failurePhase"], "fast")
        self.assertLessEqual(diagnostics["seedsAttempted"], 12)
        self.assertIsNotNone(diagnostics["bestAngles"])
        self.assertIsNotNone(diagnostics["bestPositionErrorM"])
        self.assertIsNotNone(diagnostics["bestOrientationErrorRad"])

    def test_vertical_is_preferred_then_fixed_small_tilt_is_used(self):
        class TiltOnlyRobot:
            def __init__(self):
                self.host = HostKinematicsPreviewRobot()
                self.last_coords = None

            def solve_inv_kinematics(self, coords, current):
                self.last_coords = list(coords)
                axes = kin.tool_axis_diagnostics(kin.rotation_from_rpy_deg(coords[3:6]))
                if axes["approachTiltDeg"] < 1.0:
                    return [40.0] * 6
                return self.host.solve_inv_kinematics(coords, current)

            def angles_to_coords(self, angles):
                if max(angles) - min(angles) < 0.01:
                    return list(self.last_coords)
                return self.host.angles_to_coords(angles)

        steps = []
        for state_id, z, mode in (("seq01_s1_approach", 0.16, 0), ("seq01_s2_descend", 0.08, 1)):
            tcp = (0.18, 0.02, z)
            flange, rotation = kin.top_down_flange_pose(tcp, 90.0)
            steps.append({
                "stateId": state_id,
                "coordsMm": [value * 1000.0 for value in flange] + list(kin.rpy_deg_from_rotation(rotation)),
                "targetTcpPoseM": dict(zip(("x", "y", "z"), tcp)),
                "desiredJawYawDeg": 90.0,
                "coordMode": mode,
            })
        result = RobotService(None, 115200, 0.1)._preview_coordinate_group(
            TiltOnlyRobot(), steps, [0.0, 0.0, 0.0, 0.0, 0.0, -45.0]
        )
        self.assertTrue(result["ok"], result.get("error"))
        tilts = {step["selectedOrientation"]["tiltOffsetDeg"] for step in steps}
        self.assertEqual(len(tilts), 1)
        self.assertGreater(next(iter(tilts)), 0.0)
        self.assertLessEqual(next(iter(tilts)), 10.0)

    def test_fresh_full_plan_preflight_rejects_before_home_command(self):
        class RefusingRobot:
            def __init__(self):
                self.last_coords = None
                self.home_commands = 0
                self.coordinate_commands = 0

            def solve_inv_kinematics(self, coords, current):
                self.last_coords = list(coords)
                return [12.52, -60.13, 0.0, -29.86, 0.0, -38.67]

            def angles_to_coords(self, angles):
                return list(self.last_coords)

            def get_angles(self):
                return [0.0, 0.0, 0.0, 0.0, 0.0, -45.0]

            def send_angles(self, angles, speed):
                self.home_commands += 1

            def send_coords(self, coords, speed, mode):
                self.coordinate_commands += 1

        target = [267.56, 3.68, 203.8, -180.0, 0.0, -44.88]
        flange_position = tuple(value / 1000.0 for value in target[:3])
        flange_rotation = kin.rotation_from_rpy_deg(target[3:6])
        tcp_position, _ = kin.tcp_from_flange(flange_position, flange_rotation)
        robot = RefusingRobot()
        service = RobotService("fake", 115200, 0.1)
        service.robot = robot
        plan = {
            "ok": True,
            "mode": "coordinate_program",
            "physicalReady": True,
            "coordinatePreview": {"ok": True},
            "steps": [
                {"stateId": "seq01_home", "name": "home", "robotCommand": "home"},
                {
                    "stateId": "seq02_s1_approach",
                    "name": "approach",
                    "coordsMm": target,
                    "baseToolRpyDeg": target[3:6],
                    "targetTcpPoseM": dict(zip(("x", "y", "z"), tcp_position)),
                    "desiredJawYawDeg": kin.tool_axis_diagnostics(flange_rotation)["jawYawDeg"],
                    "coordMode": 0,
                },
            ],
        }
        result = service.execute_pick_plan(plan, "RUN_PHYSICAL_PICK")
        self.assertFalse(result["ok"])
        self.assertIn("Fresh IK validation failed", result["error"])
        self.assertEqual(robot.home_commands, 0)
        self.assertEqual(robot.coordinate_commands, 0)

    def test_controller_error_codes_are_reported_without_motion(self):
        class ErrorRobot:
            def __init__(self, code):
                self.code = code

            def get_error_information(self):
                return self.code

        service = RobotService(None, 115200, 0.1)
        self.assertEqual(
            service.read_controller_error(ErrorRobot(32)),
            {"code": 32, "label": "controller_ik_no_solution"},
        )
        self.assertEqual(
            service.read_controller_error(ErrorRobot(33)),
            {"code": 33, "label": "controller_linear_no_adjacent_solution"},
        )

    def test_firmware_roundtrip_beyond_physical_envelope_is_rejected(self):
        target = [280.0, -5.239, 196.6, -180.0, 0.0, -38.808]
        angles = [12.52, -60.13, 0.0, -29.86, 0.0, -38.67]
        firmware_fk = [261.7, -6.8, 196.6, -179.99, 0.0, -38.81]
        result = RobotService._validate_firmware_ik(target, angles, [0, 0, 0, 0, 0, -45], firmware_fk)
        self.assertFalse(result["ok"])
        self.assertIn("firmware_fk_roundtrip_residual", result["rejectionReasons"])

    def test_bin_carry_may_choose_a_different_fixed_top_down_yaw(self):
        class YawSelectiveRobot:
            def solve_inv_kinematics(self, coords, current):
                # Reject the pick-aligned yaw, but provide a deterministic
                # valid result for the 90-degree carry alternative.
                if abs(((float(coords[5]) + 180.0) % 360.0) - 180.0) < 60.0:
                    return [40.0] * 6
                return HostKinematicsPreviewRobot().solve_inv_kinematics(coords, current)

            def angles_to_coords(self, angles):
                return HostKinematicsPreviewRobot.angles_to_coords(angles)

        tcp_position = (0.12, 0.12, 0.13)
        flange_position, flange_rotation = kin.top_down_flange_pose(tcp_position, -84.0)
        rpy = list(kin.rpy_deg_from_rotation(flange_rotation))
        step = {
            "stateId": "seq02_s5_carry",
            "coordsMm": [value * 1000.0 for value in flange_position] + rpy,
            "targetTcpPoseM": dict(zip(("x", "y", "z"), tcp_position)),
            "baseToolRpyDeg": rpy,
            "coordMode": 0,
        }
        result = RobotService(None, 115200, 0.1)._preview_coordinate_group(
            YawSelectiveRobot(), [step], [0.0, 0.0, 0.0, 0.0, 0.0, -45.0]
        )
        self.assertTrue(result["ok"], result.get("error"))
        self.assertNotEqual(step["selectedOrientation"]["yawOffsetDeg"], 0.0)

    def test_program_one_bin_drop_is_shifted_inside_and_toward_base(self):
        bin_obj = {
            "position": {"x": 0.2444839481, "y": 0.1579678936, "z": 0.0},
            "outer": {"x": 0.125, "y": 0.125, "z": 0.07},
            "wallThickness": 0.006,
            "orientationDeg": 0.0,
        }
        geometry = Workcell.bin_geometry(bin_obj)
        drop = Workcell.reachable_bin_drop_xy(bin_obj, geometry, {"x": 0.04, "y": 0.04})
        self.assertLess(math.hypot(drop["x"], drop["y"]), 0.26)
        self.assertAlmostEqual(drop["x"], 0.2140, places=3)
        self.assertAlmostEqual(drop["y"], 0.1275, places=3)


class CoordinatePollingTests(unittest.TestCase):
    class FeedbackRobot:
        def __init__(self, coords, moving=None):
            self.coords = list(coords)
            self.moving = list(moving or [1, 0, 0, 0, 0])

        def is_moving(self):
            return self.moving.pop(0) if self.moving else 0

        def is_in_position(self, data, id=0):
            return 0

        def get_coords(self):
            return list(self.coords)

    def test_recorded_stable_endpoint_is_accepted_with_physical_tolerance(self):
        target = [227.75, -55.83, 228.0, 179.73, 1.36, -44.65]
        actual = [227.1, -59.4, 222.3, 177.66, 2.51, -44.88]
        service = RobotService(None, 115200, 0.1)
        result = service.wait_for_motion_stop(self.FeedbackRobot(actual), target, timeout_s=2.0)
        self.assertTrue(result["reached"])
        self.assertEqual(result["completion"], "settled_near_target")
        errors = service.coords_error(
            target, actual, COORD_PHYSICAL_TOLERANCE_MM, COORD_PHYSICAL_RPY_TOLERANCE_DEG
        )
        self.assertTrue(errors["withinTolerance"])
        self.assertEqual(errors["maxPositionErrorMm"], 5.7)

    def test_stopped_endpoint_outside_physical_tolerance_is_rejected(self):
        target = [200.0, 0.0, 200.0, 180.0, 0.0, 0.0]
        actual = [180.0, 0.0, 180.0, 180.0, 0.0, 0.0]
        service = RobotService(None, 115200, 0.1)
        result = service.wait_for_motion_stop(
            self.FeedbackRobot(actual, moving=[0, 0, 0, 0, 0, 0]), target, timeout_s=2.0
        )
        self.assertFalse(result["reached"])
        self.assertEqual(result["completion"], "stopped_outside_tolerance")

    def test_recorded_angular_carry_endpoint_is_accepted(self):
        target = [214.0, 127.5, 228.0, 179.73, -8.64, 0.35]
        actual = [214.8, 125.0, 219.7, 177.99, -5.38, -0.18]
        service = RobotService(None, 115200, 0.1)
        result = service.wait_for_motion_stop(
            self.FeedbackRobot(actual),
            target,
            timeout_s=2.0,
            position_tolerance_mm=COORD_PHYSICAL_ANGULAR_TOLERANCE_MM,
            rpy_tolerance_deg=COORD_PHYSICAL_RPY_TOLERANCE_DEG,
        )
        self.assertTrue(result["reached"])
        self.assertEqual(result["completion"], "settled_near_target")
        self.assertEqual(result["maxPositionErrorMm"], 8.3)

    def test_recorded_linear_lower_endpoint_is_accepted_with_safe_envelope(self):
        target = [214.0, 127.5, 188.0, 179.73, -8.64, 0.35]
        actual = [215.3, 124.9, 174.7, 177.36, -4.77, -0.3]
        service = RobotService(None, 115200, 0.1)
        result = service.wait_for_motion_stop(
            self.FeedbackRobot(actual),
            target,
            timeout_s=2.0,
            position_tolerance_mm=COORD_PHYSICAL_TOLERANCE_MM,
            rpy_tolerance_deg=COORD_PHYSICAL_RPY_TOLERANCE_DEG,
        )
        self.assertTrue(result["reached"])
        self.assertEqual(result["completion"], "settled_near_target")
        self.assertEqual(result["maxPositionErrorMm"], 13.3)


if __name__ == "__main__":
    unittest.main()
