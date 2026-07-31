import json
import tempfile
import unittest
from pathlib import Path

from web_server import DashboardHandler
from workcell import HOME_ANGLES, Workcell


class SpatialAndTaughtPointTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cell = Workcell(Path(self.temp.name))
        self.cell.calibration["fiducials"]["referenceMarkers"] = [
            {"id": 0, "center": {"x": 0.32, "y": 0.22}, "sizeM": 0.05},
            {"id": 1, "center": {"x": 0.32, "y": -0.22}, "sizeM": 0.05},
            {"id": 2, "center": {"x": 0.08, "y": -0.22}, "sizeM": 0.05},
            {"id": 3, "center": {"x": 0.08, "y": 0.22}, "sizeM": 0.05},
        ]
        self.cell.calibration["fiducials"]["baselineHomography"] = [
            [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]
        ]

    def tearDown(self):
        self.temp.cleanup()

    def add_part(self, part_id="part-3", x=0.22, y=0.10):
        return self.cell.upsert_part({
            "id": part_id,
            "label": "Part 3",
            "type": "box",
            "position": {"x": x, "y": y, "z": 0.02},
            "size": {"x": 0.04, "y": 0.03, "z": 0.04},
            "orientationDeg": 0,
            "trackingMode": "virtual",
        })["part"]

    def add_point(self, point_id="point-inspection", tool="adaptive_gripper"):
        return self.cell.save_taught_point({
            "id": point_id,
            "label": "Inspection Point",
            "firmwareFlangeCoordsMmDeg": [200, 78, 176, 180, 0, -45],
            "jointAnglesDeg": [0, -25, 55, -30, 0, -45],
            "endEffector": tool,
            "toolCalibrationFingerprint": self.cell.tool_calibration_fingerprint(tool),
            "supportSurfaceZ": 0.0,
            "uses": ["waypoint", "destination"],
        })["point"]

    def test_right_region_always_decreases_robot_y_and_keeps_full_footprint_inside(self):
        part = self.add_part()
        result = self.cell.resolve_spatial_destination({
            "entityKind": "part", "entityId": part["id"],
            "destination": {"kind": "region", "region": "right"},
        })
        self.assertTrue(result["ok"], result)
        target = result["candidates"][0]["position"]
        self.assertLess(target["y"], part["position"]["y"])
        right = result["workspace"]["regions"]["right"]
        self.assertGreaterEqual(target["y"] - part["size"]["y"] / 2, right["yMin"])
        self.assertLessEqual(target["y"] + part["size"]["y"] / 2, right["yMax"])
        self.assertEqual(result["workspace"]["coordinateConvention"]["right"], "-Y")

    def test_relative_two_inches_right_uses_negative_y(self):
        part = self.add_part(y=0.10)
        result = self.cell.resolve_spatial_destination({
            "entityKind": "part", "entityId": part["id"],
            "destination": {"kind": "relative", "dxM": 0.0, "dyM": -0.0508},
        })
        self.assertTrue(result["ok"], result)
        self.assertAlmostEqual(result["candidates"][0]["position"]["y"], 0.0492, places=6)

    def test_robot_base_and_occupied_candidates_are_rejected(self):
        part = self.add_part(x=0.22, y=0.12)
        near_base = self.cell.resolve_spatial_destination({
            "entityKind": "part", "entityId": part["id"],
            "destination": {"kind": "relative", "dxM": -0.12, "dyM": -0.12},
        })
        self.assertFalse(near_base["ok"])
        self.assertIn("robot_base_exclusion", near_base["error"])

        self.cell.upsert_bin({
            "id": "bin-a", "label": "Bin A",
            "position": {"x": 0.22, "y": 0.02, "z": 0},
            "outer": {"x": 0.08, "y": 0.08, "z": 0.04},
        })
        blocked = self.cell.resolve_spatial_destination({
            "entityKind": "part", "entityId": part["id"],
            "destination": {"kind": "relative", "dxM": 0, "dyM": -0.10},
        })
        self.assertFalse(blocked["ok"])
        self.assertIn("occupied_by_bin:bin-a", blocked["error"])

    def test_simulation_only_bin_blocks_physical_readiness_until_confirmed(self):
        part = self.add_part()
        self.cell.upsert_bin({
            "id": "bin-a", "label": "Bin A",
            "position": {"x": 0.22, "y": -0.12, "z": 0},
            "outer": {"x": 0.10, "y": 0.10, "z": 0.04},
            "positionStatus": "simulation_only",
        })
        plan = self.cell.plan_program(
            [{"type": "pick", "objectId": part["id"]}, {"type": "place", "binId": "bin-a"}],
            HOME_ANGLES,
            "simulated bin",
        )
        self.assertTrue(plan["ok"], plan)
        self.assertFalse(plan["physicalReady"])
        self.assertEqual(plan["unverifiedDestinations"][0]["id"], "bin-a")
        self.assertIn("simulated position", self.cell.validate_plan_object_snapshots(plan))

        self.cell.confirm_bin_position("bin-a")
        confirmed = self.cell.plan_program(
            [{"type": "pick", "objectId": part["id"]}, {"type": "place", "binId": "bin-a"}],
            HOME_ANGLES,
            "confirmed bin",
        )
        self.assertTrue(confirmed["physicalReady"])

    def test_spatial_command_tools_build_programs_without_model_coordinates(self):
        part = self.add_part()
        self.cell.upsert_bin({
            "id": "bin-a", "label": "Bin A",
            "position": {"x": 0.24, "y": 0.17, "z": 0},
            "outer": {"x": 0.10, "y": 0.10, "z": 0.04},
        })
        point = self.add_point()
        handler = DashboardHandler.__new__(DashboardHandler)
        handler.scene = self.cell

        def fast_validated_plan(body):
            plan = self.cell.plan_program(
                body.get("steps") or [], HOME_ANGLES, body.get("name") or "test"
            )
            if plan.get("ok"):
                plan["coordinatePreview"] = {"ok": True, "states": [], "solvedStates": 0}
            return plan

        handler.plan_program_request = fast_validated_plan
        region = handler.plan_spatial_move({
            "objectId": part["id"], "destinationKind": "region", "region": "right"
        })
        self.assertTrue(region["ok"], region)
        self.assertLess(region["spatialResolution"]["selectedPosition"]["y"], 0.0)
        self.assertEqual(region["program"]["steps"][1]["type"], "place")

        virtual_move = handler.update_virtual_layout({
            "binId": "bin-a", "destinationKind": "region", "region": "right"
        })
        self.assertTrue(virtual_move["ok"], virtual_move)
        self.assertTrue(virtual_move["simulationOnly"])
        self.assertEqual(self.cell.bins["bin-a"]["positionStatus"], "simulation_only")
        into_bin = handler.plan_spatial_move({
            "objectId": part["id"], "destinationKind": "bin", "binId": "bin-a"
        })
        self.assertTrue(into_bin["ok"], into_bin)
        self.assertFalse(into_bin["plan"]["physicalReady"])

        point_move = handler.plan_move_to_point({"pointId": point["id"]})
        self.assertTrue(point_move["ok"], point_move)
        self.assertEqual(point_move["program"]["steps"], [
            {"type": "move_to_point", "pointId": point["id"]}
        ])

    def test_spatial_context_and_planner_explain_hidden_registered_part(self):
        bound = self.cell.bind_tagged_part({
            "partId": "part-hidden", "tagId": 11, "label": "Part 3",
            "size": {"x": 0.04, "y": 0.03, "z": 0.04},
        })
        self.assertTrue(bound["ok"], bound)
        context = self.cell.spatial_context()
        warning = next(item for item in context["availabilityWarnings"] if item["entityId"] == "part-hidden")
        self.assertEqual(warning["code"], "tag_not_visible")

        handler = DashboardHandler.__new__(DashboardHandler)
        handler.scene = self.cell
        result = handler.plan_spatial_move({
            "objectQuery": "Part 3", "destinationKind": "region", "region": "right",
        })
        self.assertFalse(result["ok"])
        self.assertIn("AprilTag is not currently visible", result["error"])

    def test_taught_point_persists_and_replays_measured_pose_with_seed(self):
        point = self.add_point()
        reloaded = Workcell(Path(self.temp.name))
        self.assertEqual(reloaded.taught_points[point["id"]]["jointAnglesDeg"], point["jointAnglesDeg"])
        plan = reloaded.plan_program(
            [{"type": "move_to_point", "pointId": point["id"]}], HOME_ANGLES, "inspection"
        )
        self.assertTrue(plan["ok"], plan)
        arrive = next(step for step in plan["steps"] if step["name"] == "move_to_point")
        self.assertEqual(arrive["coordsMm"], point["firmwareFlangeCoordsMmDeg"])
        self.assertEqual(arrive["preferredJointSeedDeg"], point["jointAnglesDeg"])
        self.assertEqual(arrive["orientationPolicy"], "fixed_taught_pose")

    def test_taught_point_rejects_invalid_support_surface_instead_of_clamping(self):
        payload = {
            "label": "Bad Surface",
            "firmwareFlangeCoordsMmDeg": [200, 78, 176, 180, 0, -45],
            "jointAnglesDeg": [0, -25, 55, -30, 0, -45],
            "endEffector": "adaptive_gripper",
            "toolCalibrationFingerprint": self.cell.tool_calibration_fingerprint("adaptive_gripper"),
            "supportSurfaceZ": "not-a-number",
        }
        result = self.cell.save_taught_point(payload)
        self.assertFalse(result["ok"])
        self.assertIn("finite number", result["error"])
        payload["supportSurfaceZ"] = 0.31
        result = self.cell.save_taught_point(payload)
        self.assertFalse(result["ok"])
        self.assertIn("between 0.0 and 0.30", result["error"])

    def test_taught_point_rejects_wrong_tool_and_changed_calibration(self):
        point = self.add_point()
        self.cell.end_effector = "suction_gripper"
        wrong_tool = self.cell.plan_program(
            [{"type": "move_to_point", "pointId": point["id"]}], HOME_ANGLES, "wrong tool"
        )
        self.assertFalse(wrong_tool["ok"])
        self.assertIn("captured with", wrong_tool["error"])

        self.cell.end_effector = "adaptive_gripper"
        self.cell.coordinate_planner["toolProfiles"]["adaptive_gripper"]["tcpCorrectionLocalM"]["x"] = 0.002
        changed = self.cell.plan_program(
            [{"type": "move_to_point", "pointId": point["id"]}], HOME_ANGLES, "changed tool"
        )
        self.assertFalse(changed["ok"])
        self.assertIn("recapture", changed["error"])

    def test_taught_point_rejects_bad_joints_and_inconsistent_pose(self):
        bad_joint = self.cell.save_taught_point({
            "label": "Bad Joint",
            "firmwareFlangeCoordsMmDeg": [200, 0, 180, 180, 0, 0],
            "jointAnglesDeg": [999, 0, 0, 0, 0, 0],
            "endEffector": "adaptive_gripper",
        })
        self.assertFalse(bad_joint["ok"])
        self.assertIn("J1", bad_joint["error"])
        inconsistent = self.cell.save_taught_point({
            "label": "Bad TCP",
            "firmwareFlangeCoordsMmDeg": [200, 0, 180, 180, 0, 0],
            "jointAnglesDeg": [0, 0, 0, 0, 0, 0],
            "endEffector": "adaptive_gripper",
            "tcpPoseM": {
                "position": {"x": 0.40, "y": 0.40, "z": 0.40},
                "rpyDeg": {"rx": 0, "ry": 0, "rz": 0},
            },
        })
        self.assertFalse(inconsistent["ok"])
        self.assertIn("inconsistent", inconsistent["error"])

    def test_taught_destination_and_bin_must_contain_complete_footprint(self):
        part = self.add_part()
        point = self.add_point("edge-point")
        self.cell.taught_points[point["id"]]["tcpPoseM"]["position"]["y"] = 0.219
        outside = self.cell.resolve_spatial_destination({
            "entityKind": "part", "entityId": part["id"],
            "destination": {"kind": "point", "pointId": point["id"]},
        })
        self.assertFalse(outside["ok"])
        self.assertIn("complete object footprint", outside["error"])
        self.cell.upsert_bin({
            "id": "tiny-bin", "label": "Tiny Bin",
            "position": {"x": 0.25, "y": -0.12, "z": 0},
            "outer": {"x": 0.04, "y": 0.04, "z": 0.03},
            "wallThickness": 0.008,
        })
        too_small = self.cell.resolve_spatial_destination({
            "entityKind": "part", "entityId": part["id"],
            "destination": {"kind": "bin", "binId": "tiny-bin"},
        })
        self.assertFalse(too_small["ok"])
        self.assertIn("fit completely", too_small["error"])

    def test_point_destination_uses_support_surface_and_tool_actions_are_normalized(self):
        part = self.add_part()
        point = self.add_point()
        saved = self.cell.save_program({
            "name": "Custom point sequence",
            "steps": [
                {"type": "acquire"},
                {"type": "release"},
                {"type": "pick", "objectId": part["id"]},
                {"type": "place", "pointId": point["id"]},
            ],
        })
        self.assertTrue(saved["ok"], saved)
        self.assertEqual([step["type"] for step in saved["program"]["steps"]], [
            "acquire", "release", "pick", "place"
        ])
        plan = self.cell.plan_program(saved["program"]["steps"], HOME_ANGLES, saved["program"]["name"])
        self.assertTrue(plan["ok"], plan)
        self.assertTrue(any(step.get("gripperAction") == "auto_grip" for step in plan["steps"]))
        self.assertTrue(any(snapshot.get("kind") == "point" for snapshot in plan["destinationSnapshots"]))

    def test_only_successful_virtual_part_execution_persists_a_drop(self):
        virtual = self.add_part("virtual-part", x=0.22, y=0.10)
        executed = [{
            "releaseObjectId": virtual["id"],
            "placedPosition": {"x": 0.25, "y": -0.08, "z": 0.02},
        }]
        self.cell.apply_executed_steps(executed, physical_run_ok=False)
        self.assertAlmostEqual(self.cell.parts[virtual["id"]]["position"]["y"], 0.10)
        self.cell.apply_executed_steps(executed, physical_run_ok=True)
        self.assertAlmostEqual(self.cell.parts[virtual["id"]]["position"]["y"], -0.08)

        tagged = self.add_part("tagged-part", x=0.22, y=0.12)
        self.cell.parts[tagged["id"]]["trackingMode"] = "apriltag"
        self.cell.parts[tagged["id"]]["source"] = "camera"
        tagged_execution = [{
            "releaseObjectId": tagged["id"],
            "placedPosition": {"x": 0.24, "y": -0.10, "z": 0.02},
        }]
        self.cell.apply_executed_steps(tagged_execution, physical_run_ok=True)
        self.assertAlmostEqual(self.cell.parts[tagged["id"]]["position"]["y"], 0.12)


class RealtimeConfigurationTests(unittest.TestCase):
    def test_realtime_uses_21_low_reasoning_and_push_to_talk(self):
        root = Path(__file__).resolve().parents[1]
        server = (root / "web_server.py").read_text()
        client = (root / "static" / "js" / "realtime.js").read_text()
        example = (root / "api_keys.env.example").read_text()
        self.assertIn('"gpt-realtime-2.1"', server)
        self.assertIn('"reasoning": {"effort": "low"}', server)
        self.assertIn('"max_output_tokens": 160', server)
        self.assertIn('"turn_detection": None', server)
        self.assertIn("OPENAI_REALTIME_MODEL=gpt-realtime-2.1", example)
        self.assertIn("track.enabled = false", client)
        self.assertLess(client.index('type: "input_audio_buffer.clear"'), client.index("track.enabled = true"))
        self.assertLess(client.index('type: "input_audio_buffer.commit"'), client.index("requestResponseCreate();", client.index("function endTalk")))
        self.assertIn("MIN_PUSH_TO_TALK_MS = 200", client)
        self.assertIn("if (realtime.responseActive)", client)
        self.assertNotIn('event.type === "response.output_item.done"', client)
        self.assertIn('event.type === "response.done"', client)
        self.assertIn("AI plan could not be created.", client)
        self.assertIn("Persistent taught points can only be created from measured robot capture", server)
        self.assertIn("NEVER speak before calling a tool", server)


if __name__ == "__main__":
    unittest.main()
