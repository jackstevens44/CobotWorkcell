import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

from mycobot_kinematics import firmware_flange_kinematics, rpy_deg_from_rotation
from web_server import DashboardHandler, RobotService
from workcell import HOME_ANGLES, Workcell


class ProgramSchemaV2Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cell = Workcell(Path(self.temp.name))
        angles = [0, -25, 55, -30, 0, -45]
        position, rotation = firmware_flange_kinematics(angles)
        rpy = rpy_deg_from_rotation(rotation)
        self.point = self.cell.save_taught_point({
            "id": "inspection",
            "label": "Inspection",
            "firmwareFlangeCoordsMmDeg": [
                position[0] * 1000, position[1] * 1000, position[2] * 1000,
                rpy[0], rpy[1], rpy[2],
            ],
            "jointAnglesDeg": angles,
            "endEffector": "adaptive_gripper",
            "toolCalibrationFingerprint": self.cell.tool_calibration_fingerprint("adaptive_gripper"),
            "uses": ["waypoint", "destination"],
        })["point"]

    def tearDown(self):
        self.temp.cleanup()

    def test_v2_program_persists_stable_nodes_repeat_and_disabled_steps(self):
        result = self.cell.save_program({
            "editorVersion": 2,
            "name": "Industrial sequence",
            "repeatCount": 3,
            "steps": [
                {
                    "id": "move-1", "type": "move", "motionType": "joint",
                    "pointId": self.point["id"], "speed": 12,
                },
                {"id": "wait-1", "type": "wait", "durationMs": 750},
                {"id": "tool-1", "type": "tool", "action": "acquire", "enabled": False},
                {"id": "home-1", "type": "home"},
            ],
        })
        self.assertTrue(result["ok"], result)
        program = result["program"]
        self.assertEqual(program["editorVersion"], 2)
        self.assertEqual(program["repeatCount"], 3)
        self.assertEqual([step["id"] for step in program["steps"]], [
            "move-1", "wait-1", "tool-1", "home-1",
        ])
        reloaded = Workcell(Path(self.temp.name))
        saved = reloaded.programs[program["id"]]
        self.assertEqual(saved["repeatCount"], 3)
        self.assertFalse(saved["steps"][2]["enabled"])

    def test_joint_motion_uses_measured_angles_and_repeats_complete_program(self):
        plan = self.cell.plan_program([
            {
                "id": "move-1", "type": "move", "motionType": "joint",
                "pointId": self.point["id"], "speed": 9,
            },
            {"id": "wait-1", "type": "wait", "durationMs": 100},
        ], HOME_ANGLES, "repeat", repeat_count=2)
        self.assertTrue(plan["ok"], plan)
        joint_moves = [step for step in plan["steps"] if step.get("jointTargetDeg")]
        self.assertEqual(len(joint_moves), 2)
        self.assertEqual(joint_moves[0]["jointTargetDeg"], self.point["jointAnglesDeg"])
        self.assertEqual(joint_moves[0]["sourceStepId"], "move-1")
        self.assertEqual([step["sourceIteration"] for step in joint_moves], [1, 2])
        self.assertEqual(len([step for step in plan["steps"] if step.get("waitMs")]), 2)

        service = RobotService(None, 115200, 0.1)
        previewed = service.add_coordinate_preview(plan, HOME_ANGLES)
        self.assertTrue(previewed["coordinatePreview"]["ok"], previewed["coordinatePreview"])
        self.assertGreater(len(joint_moves[0]["trajectory"]), 1)
        self.assertEqual(joint_moves[0]["trajectory"][-1]["angles"], self.point["jointAnglesDeg"])

    def test_embedded_waypoint_is_supported_without_creating_global_point(self):
        embedded = dict(self.point)
        embedded["id"] = "embedded"
        plan = self.cell.plan_program([{
            "id": "linear-1", "type": "move", "motionType": "linear",
            "waypoint": embedded, "speed": 7,
        }], HOME_ANGLES, "embedded")
        self.assertTrue(plan["ok"], plan)
        linear = next(step for step in plan["steps"] if step.get("coordsMm"))
        self.assertEqual(linear["name"], "linear_move")
        self.assertEqual(linear["coordMode"], 1)
        self.assertNotIn("embedded", self.cell.taught_points)

    def test_program_delete_reports_missing_id_and_persists(self):
        saved = self.cell.save_program({
            "editorVersion": 2,
            "name": "Delete me",
            "steps": [{"id": "home-1", "type": "home"}],
        })
        program_id = saved["program"]["id"]
        missing = self.cell.delete_program("does-not-exist")
        self.assertFalse(missing["ok"])
        self.assertIn("not found", missing["error"])
        deleted = self.cell.delete_program(program_id)
        self.assertTrue(deleted["ok"], deleted)
        self.assertEqual(deleted["deletedProgramId"], program_id)
        self.assertNotIn(program_id, Workcell(Path(self.temp.name)).programs)

    def test_embedded_joint_and_linear_waypoints_do_not_require_global_points(self):
        embedded = dict(self.point)
        embedded["id"] = "83d2335d-1834-45df-95f2-0e8016ae389e-waypoint"
        for motion_type in ("joint", "linear"):
            plan = self.cell.plan_program([{
                "id": f"{motion_type}-step",
                "type": "move",
                "motionType": motion_type,
                "waypoint": embedded,
                "speed": 10,
            }], HOME_ANGLES, f"embedded-{motion_type}")
            self.assertTrue(plan["ok"], plan)
            self.assertFalse(any(
                snapshot.get("id") == embedded["id"]
                for snapshot in plan["destinationSnapshots"]
            ))
            motion = next(
                step for step in plan["steps"]
                if step.get("jointTargetDeg") is not None or step.get("coordsMm") is not None
            )
            self.assertEqual(motion["waypointSource"], "embedded")
            self.assertEqual(motion["waypointId"], embedded["id"])
            self.assertIsNone(self.cell.validate_plan_object_snapshots(plan), plan)

    def test_embedded_waypoint_rejects_changed_tool_calibration_with_step_id(self):
        embedded = dict(self.point)
        embedded["id"] = "embedded-calibration"
        plan = self.cell.plan_program([{
            "id": "joint-step",
            "type": "move",
            "motionType": "joint",
            "waypoint": embedded,
        }], HOME_ANGLES, "embedded")
        plan["steps"][0]["capturedToolCalibrationFingerprint"] = "outdated"
        error = self.cell.validate_plan_object_snapshots(plan)
        self.assertIn("Embedded waypoint", error)
        self.assertIn("joint-step", error)
        self.assertIn("outdated tool calibration", error)

    def test_linked_waypoint_still_rejects_deletion(self):
        plan = self.cell.plan_program([{
            "id": "linked-step",
            "type": "move",
            "motionType": "joint",
            "pointId": self.point["id"],
        }], HOME_ANGLES, "linked")
        self.assertTrue(any(
            snapshot.get("id") == self.point["id"]
            for snapshot in plan["destinationSnapshots"]
        ))
        self.cell.delete_taught_point(self.point["id"])
        error = self.cell.validate_plan_object_snapshots(plan)
        self.assertIn("Linked taught point", error)
        self.assertIn("deleted", error)


class JogSafetyTests(unittest.TestCase):
    def setUp(self):
        self.service = RobotService(None, 115200, 0.1)
        self.robot = MagicMock()
        self.robot.get_angles.return_value = [0, 0, 0, 0, 0, -45]
        self.service.robot = self.robot
        self.service.get_robot_locked = MagicMock(return_value=self.robot)

    def tearDown(self):
        self.service.stop_jog()

    def test_hold_jog_has_session_heartbeat_and_explicit_stop(self):
        started = self.service.start_joint_jog(2, 1, 10)
        self.assertTrue(started["ok"], started)
        session_id = started["jog"]["sessionId"]
        self.robot.jog_angle.assert_called_once_with(2, 1, 10)
        self.assertTrue(self.service.heartbeat_jog(session_id)["ok"])
        self.assertFalse(self.service.heartbeat_jog("wrong")["ok"])
        self.assertTrue(self.service.stop_jog()["ok"])
        self.robot.jog_stop.assert_called()

    def test_suction_j6_is_locked_for_hold_and_increment(self):
        self.service.set_end_effector("suction_gripper")
        self.assertFalse(self.service.start_joint_jog(6, 1, 10)["ok"])
        self.assertFalse(self.service.step_jog("joint", 6, 1, 10)["ok"])
        self.robot.jog_angle.assert_not_called()
        self.robot.jog_increment_angle.assert_not_called()

    def test_increment_limits_are_enforced_before_driver_call(self):
        self.assertFalse(self.service.step_jog("joint", 1, 5.1, 10)["ok"])
        self.assertFalse(self.service.step_jog("tcp", 1, 10.1, 10)["ok"])
        self.assertFalse(self.service.step_jog("tcp", 4, 5.1, 10)["ok"])
        self.robot.jog_increment_angle.assert_not_called()
        self.robot.jog_increment_coord.assert_not_called()


class PhysicalProgramDispatchTests(unittest.TestCase):
    class ImmediateRobot:
        def __init__(self):
            self.angles = list(HOME_ANGLES)
            self.angle_commands = []

        def get_angles(self, **_kwargs):
            return list(self.angles)

        def send_angles(self, angles, speed):
            self.angles = [float(value) for value in angles]
            self.angle_commands.append((list(self.angles), int(speed)))

    def test_preview_trajectory_on_joint_move_is_executable(self):
        robot = self.ImmediateRobot()
        service = RobotService("fake", 115200, 0.1)
        service.robot = robot
        target = [5.0, -10.0, 15.0, -5.0, 2.0, -40.0]
        plan = {
            "ok": True,
            "mode": "coordinate_program",
            "physicalReady": True,
            "coordinatePreview": {"ok": True},
            "steps": [{
                "stateId": "seq01_joint",
                "name": "joint_move",
                "sourceStepId": "move-1",
                "jointTargetDeg": target,
                "speed": 12,
                # The v2 simulator adds this path. It must not make the
                # otherwise-valid Joint Move look like a legacy program.
                "trajectory": [
                    {"t": 0.5, "angles": [2.5, -5.0, 7.5, -2.5, 1.0, -42.5]},
                    {"t": 1.0, "angles": target},
                ],
            }],
        }
        result = service.execute_pick_plan(plan, "RUN_PHYSICAL_PICK")
        self.assertTrue(result["ok"], result)
        self.assertEqual(robot.angle_commands, [(target, 12)])
        self.assertEqual(result["executedSteps"][0]["sourceStepId"], "move-1")

    def test_server_speed_override_is_bounded_and_does_not_mutate_preview(self):
        original = {
            "steps": [
                {"robotCommand": "home"},
                {"jointTargetDeg": [0] * 6, "speed": 20},
                {"coordsMm": [200, 0, 200, 180, 0, 0], "coordSpeed": 30},
                {"waitMs": 1000},
            ],
        }
        scaled = DashboardHandler.plan_with_speed_override(original, 50)
        self.assertNotIn("speed", original["steps"][0])
        self.assertEqual(scaled["steps"][0]["speed"], 8)
        self.assertEqual(scaled["steps"][1]["speed"], 10)
        self.assertEqual(scaled["steps"][2]["coordSpeed"], 15)
        self.assertEqual(scaled["steps"][3]["waitMs"], 1000)
        self.assertEqual(scaled["speedOverridePct"], 50)

        minimum = DashboardHandler.plan_with_speed_override(original, -10)
        maximum = DashboardHandler.plan_with_speed_override(original, 500)
        self.assertEqual(minimum["steps"][1]["speed"], 1)
        self.assertEqual(maximum["steps"][1]["speed"], 20)

    def test_mixed_program_dispatches_every_supported_low_level_command(self):
        robot = self.ImmediateRobot()
        service = RobotService("fake", 115200, 0.1)
        service.robot = robot
        joint_target = [4.0, -8.0, 12.0, -4.0, 1.0, -42.0]
        coordinate_target = [200.0, 0.0, 200.0, 180.0, 0.0, 0.0]
        plan = {
            "ok": True,
            "mode": "coordinate_program",
            "physicalReady": True,
            "coordinatePreview": {"ok": True},
            "steps": [
                {"stateId": "home", "sourceStepId": "home-1", "robotCommand": "home"},
                {
                    "stateId": "joint", "sourceStepId": "move-1",
                    "jointTargetDeg": joint_target, "trajectory": [{"t": 1, "angles": joint_target}],
                },
                {
                    "stateId": "linear", "sourceStepId": "move-2",
                    "coordsMm": coordinate_target, "coordMode": 1,
                },
                {
                    "stateId": "tool", "sourceStepId": "tool-1",
                    "gripperAction": "auto_grip", "gripperActionTiming": "after_arrival",
                },
                {"stateId": "wait", "sourceStepId": "wait-1", "waitMs": 50},
            ],
        }
        service.add_coordinate_preview = MagicMock(side_effect=lambda candidate, _angles: candidate)
        service.run_coordinate_step = MagicMock(return_value=({
            "command": "send_coords",
            "completion": "target_reached",
            "targetCoords": coordinate_target,
            "actualAngles": joint_target,
        }, None))
        service.run_gripper_action = MagicMock(return_value={
            "action": "auto_grip", "feedback": "idle",
        })

        result = service.execute_pick_plan(plan, "RUN_PHYSICAL_PICK")
        self.assertTrue(result["ok"], result)
        self.assertEqual([item["stateId"] for item in result["executedSteps"]], [
            "home", "joint", "linear", "tool", "wait",
        ])
        self.assertEqual(len(robot.angle_commands), 2)
        service.run_coordinate_step.assert_called_once()
        service.run_gripper_action.assert_called_once_with(
            robot, "auto_grip", program_mode=True,
        )
        self.assertEqual(result["executedSteps"][-1]["motion"]["completion"], "wait_complete")


class ProgrammerFrontendContractTests(unittest.TestCase):
    def test_full_screen_workspace_replaces_legacy_button_strip(self):
        root = Path(__file__).resolve().parents[1]
        html = (root / "static/index.html").read_text()
        ui = (root / "static/js/ui.js").read_text()
        self.assertIn('id="programWorkspace"', html)
        self.assertIn('id="programViewportHost"', html)
        self.assertIn('data-command="joint"', html)
        self.assertIn('data-command="linear"', html)
        self.assertNotIn('id="addPickBtn"', html)
        self.assertIn("/api/robot/jog/start", ui)
        self.assertIn("/api/robot/jog/heartbeat", ui)
        self.assertIn("/api/robot/points/capture", ui)

    def test_program_header_actions_and_actionable_run_contract(self):
        root = Path(__file__).resolve().parents[1]
        html = (root / "static/index.html").read_text()
        ui = (root / "static/js/ui.js").read_text()
        css = (root / "static/styles.css").read_text()
        header = html.split('<header class="program-workspace-header">', 1)[1].split("</header>", 1)[0]
        footer = html.split('<footer class="program-workspace-footer">', 1)[1].split("</footer>", 1)[0]
        self.assertEqual(html.count('id="saveProgramBtn"'), 1)
        self.assertLess(header.index('id="saveProgramBtn"'), header.index('id="deleteProgramBtn"'))
        self.assertLess(header.index('id="deleteProgramBtn"'), header.index('id="closeProgrammerBtn"'))
        self.assertNotIn('id="saveProgramBtn"', footer)
        self.assertNotIn('className = "program-list-delete danger"', ui)
        self.assertNotIn("deleteProgram(program.id)", ui)
        self.assertIn("deleteProgram(state.activeProgramId)", ui)
        self.assertIn("function physicalRunBlocker()", ui)
        self.assertIn('"Blocked — View Issue"', ui)
        self.assertEqual(ui.count('post("/api/program/execute"'), 1)
        self.assertIn("speedOverridePct", ui)
        self.assertNotIn(".program-list-delete", css)

    def test_programmer_contains_long_status_and_observes_viewport_resize(self):
        root = Path(__file__).resolve().parents[1]
        ui = (root / "static/js/ui.js").read_text()
        viewport = (root / "static/js/viewport.js").read_text()
        css = (root / "static/styles.css").read_text()
        self.assertIn("Simulation complete.", ui)
        self.assertIn("Simulating ${command}", ui)
        self.assertNotIn("rendered jaw XY", ui)
        self.assertIn('new ResizeObserver(() => resizeRenderer())', viewport)
        self.assertIn("viewportResizeObserver.observe(viewport)", viewport)
        self.assertIn("if (width < 2 || height < 2) return", viewport)
        self.assertIn(".program-workspace-shell {", css)
        self.assertIn("overflow-wrap: anywhere", css)
        self.assertIn(".program-run-status {", css)
        self.assertIn("contain: inline-size", css)
        self.assertIn("flex: 1 1 0", css)
        self.assertIn("width: 0", css)


if __name__ == "__main__":
    unittest.main()
