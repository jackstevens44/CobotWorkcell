import io
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock

from camera_service import CameraService
from mycobot_driver import _valid_sextuple
from web_server import DashboardHandler, RobotService, SECURITY_RESPONSE_HEADERS, is_loopback_bind_host
from workcell import Workcell


class PersistenceRecoveryTests(unittest.TestCase):
    def test_non_object_workcell_file_falls_back_to_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            Path(folder, "workcell.json").write_text("[]")
            cell = Workcell(Path(folder))
            self.assertEqual(cell.parts, {})
            self.assertEqual(cell.registered_parts, {})
            self.assertEqual(cell.end_effector, "adaptive_gripper")

    def test_malformed_collections_and_nested_config_do_not_crash_startup(self):
        with tempfile.TemporaryDirectory() as folder:
            Path(folder, "workcell.json").write_text(json.dumps({
                "parts": [None, "bad", {"id": "part-ok", "label": "Kept"}],
                "registeredParts": [
                    {"partId": "bad-tag", "tagId": "not-a-number"},
                    {"partId": "good-tag", "tagId": 10},
                ],
                "bins": "not-a-list",
                "programs": [42],
                "calibration": {"fiducials": []},
                "camera": {"localization": [], "workspaceBounds": "bad"},
                "coordinatePlanner": {"toolProfiles": {"adaptive_gripper": []}},
                "version": "bad",
                "counter": None,
            }))
            cell = Workcell(Path(folder))
            self.assertIn("part-ok", cell.parts)
            self.assertNotIn("bad-tag", cell.registered_parts)
            self.assertIn("good-tag", cell.registered_parts)
            self.assertEqual(cell.version, 0)
            self.assertEqual(cell._counter, 1)

    def test_match_part_is_safe_for_direct_and_nested_locked_callers(self):
        with tempfile.TemporaryDirectory() as folder:
            cell = Workcell(Path(folder))
            cell.upsert_part({"id": "blue-cube", "label": "Blue Cube"})
            self.assertEqual(cell.match_part("blue")["id"], "blue-cube")
            with cell.lock:
                self.assertEqual(cell.match_part("cube")["id"], "blue-cube")

    def test_hidden_registered_pick_is_never_substituted_with_visible_part(self):
        with tempfile.TemporaryDirectory() as folder:
            cell = Workcell(Path(folder))
            cell.bind_tagged_part({
                "partId": "hidden-tagged", "tagId": 10, "label": "Hidden Tagged",
                "size": {"x": 0.04, "y": 0.04, "z": 0.04},
            })
            cell.upsert_part({"id": "visible-other", "label": "Visible Other"})
            result = cell.plan_program(
                [
                    {"type": "pick", "objectId": "hidden-tagged"},
                    {"type": "place", "position": {"x": 0.2, "y": 0.0, "z": 0.0}},
                ],
                [0.0] * 6,
            )
            self.assertFalse(result["ok"])
            self.assertIn("AprilTag is not currently visible", result["error"])
            self.assertNotIn("Visible Other", result["error"])


class CameraFreshnessTests(unittest.TestCase):
    def test_stopped_camera_never_returns_cached_frame(self):
        service = CameraService({"staleAfterS": 3.0})
        service.running = True
        service._jpeg = b"frame"
        service.last_frame_at = time.time()
        self.assertEqual(service.get_jpeg(), b"frame")
        service.stop()
        self.assertIsNone(service.get_jpeg())
        self.assertIsNone(service.last_frame_at)

    def test_running_camera_rejects_stale_frame(self):
        service = CameraService({"staleAfterS": 0.5})
        service.running = True
        service._jpeg = b"old"
        service.last_frame_at = time.time() - 2.0
        self.assertIsNone(service.get_jpeg())


class ApiInputTests(unittest.TestCase):
    @staticmethod
    def handler_with_body(raw: bytes):
        handler = object.__new__(DashboardHandler)
        handler.headers = {"Content-Length": str(len(raw))}
        handler.rfile = io.BytesIO(raw)
        return handler

    def test_json_body_must_be_an_object(self):
        with self.assertRaisesRegex(ValueError, "must be an object"):
            self.handler_with_body(b"[]").read_json()

    def test_json_body_rejects_non_finite_numbers(self):
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.handler_with_body(b'{"value": Infinity}').read_json()

    def test_server_bind_is_strictly_loopback_only(self):
        for host in ("127.0.0.1", "127.0.0.2", "localhost"):
            self.assertTrue(is_loopback_bind_host(host), host)
        for host in ("0.0.0.0", "::", "::1", "[::1]", "192.168.1.10", "cobot.local", "", None):
            self.assertFalse(is_loopback_bind_host(host), host)

    def test_same_origin_json_and_realtime_sdp_posts_are_allowed(self):
        self.assertIsNone(DashboardHandler.post_request_security_error(
            "/api/program/execute",
            {
                "Host": "127.0.0.1:8768",
                "Origin": "http://127.0.0.1:8768",
                "Sec-Fetch-Site": "same-origin",
                "Content-Type": "application/json; charset=utf-8",
            },
        ))
        self.assertIsNone(DashboardHandler.post_request_security_error(
            "/api/realtime/session",
            {
                "Host": "localhost:8768",
                "Origin": "http://localhost:8768",
                "Sec-Fetch-Site": "same-origin",
                "Content-Type": "application/sdp",
            },
        ))

    def test_cross_site_and_browser_simple_posts_are_rejected(self):
        cross_site = DashboardHandler.post_request_security_error(
            "/api/program/execute",
            {
                "Host": "127.0.0.1:8768",
                "Origin": "https://malicious.example",
                "Sec-Fetch-Site": "cross-site",
                "Content-Type": "text/plain",
            },
        )
        self.assertEqual(cross_site[0], 403)
        wrong_origin = DashboardHandler.post_request_security_error(
            "/api/program/execute",
            {
                "Host": "127.0.0.1:8768",
                "Origin": "http://localhost:8768",
                "Content-Type": "application/json",
            },
        )
        self.assertEqual(wrong_origin[0], 403)
        simple_post = DashboardHandler.post_request_security_error(
            "/api/program/execute",
            {"Host": "127.0.0.1:8768", "Content-Type": "text/plain"},
        )
        self.assertEqual(simple_post[0], 415)

    def test_security_headers_block_embedding_and_cross_origin_resources(self):
        self.assertEqual(SECURITY_RESPONSE_HEADERS["X-Frame-Options"], "DENY")
        self.assertEqual(
            SECURITY_RESPONSE_HEADERS["Cross-Origin-Resource-Policy"], "same-origin"
        )
        self.assertEqual(SECURITY_RESPONSE_HEADERS["X-Content-Type-Options"], "nosniff")


class VoiceExecutionGateTests(unittest.TestCase):
    def setUp(self):
        DashboardHandler.realtime_plans = {}
        DashboardHandler.realtime_pending_runs = {}
        self.handler = object.__new__(DashboardHandler)
        self.handler.scene = MagicMock()
        self.handler.service = MagicMock()

    def tearDown(self):
        DashboardHandler.realtime_plans = {}
        DashboardHandler.realtime_pending_runs = {}

    def test_legacy_direct_plan_execution_is_disabled(self):
        result = self.handler.execute_realtime_plan({"answer": "yes", "confirm": "RUN_PHYSICAL_PICK"})
        self.assertFalse(result["ok"])
        self.assertIn("direct-execution tool is disabled", result["error"])
        self.handler.service.execute_pick_plan.assert_not_called()

    def test_legacy_saved_program_tool_only_stages_a_run(self):
        self.handler.request_voice_program_run = MagicMock(return_value={"ok": True, "pendingRunId": "pending"})
        result = self.handler.execute_saved_program({"programId": "p1", "answer": "yes", "confirm": "RUN_PHYSICAL_PICK"})
        self.assertEqual(result["pendingRunId"], "pending")
        self.handler.request_voice_program_run.assert_called_once()
        self.handler.service.execute_pick_plan.assert_not_called()

    def test_confirmation_requires_exact_pending_run_id(self):
        result = self.handler.confirm_voice_program_run({"answer": "yes"})
        self.assertFalse(result["ok"])
        self.assertIn("pendingRunId is required", result["error"])

    def test_temporary_home_preview_can_stage_by_realtime_plan_id(self):
        plan = {
            "ok": True,
            "program": "voice home",
            "steps": [{"stateId": "seq01_home", "name": "home", "robotCommand": "home"}],
        }
        remembered = self.handler.remember_realtime_plan(plan, "plan_home_zero")
        result = self.handler.request_voice_program_run({
            "realtimePlanId": remembered["realtimePlanId"]
        })
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["program"]["temporaryPreview"])
        self.assertEqual(result["plan"], self.handler.realtime_plan_payload(plan))
        self.assertEqual(result["confirmationPrompt"], "Run voice home now?")

    def test_run_that_falls_back_to_most_recent_preview(self):
        older = self.handler.remember_realtime_plan(
            {"ok": True, "program": "older", "steps": []}, "test"
        )
        newer = self.handler.remember_realtime_plan(
            {"ok": True, "program": "newer", "steps": []}, "test"
        )
        DashboardHandler.realtime_plans[older["realtimePlanId"]]["createdAt"] -= 1.0
        result = self.handler.request_voice_program_run({})
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["program"]["realtimePlanId"], newer["realtimePlanId"])
        self.assertEqual(result["program"]["name"], "newer")

    def test_no_cancels_and_consumes_pending_run(self):
        plan = {"objectSnapshots": []}
        DashboardHandler.realtime_pending_runs["run-1"] = {
            "program": {"id": "p1"}, "plan": plan, "createdAt": time.time(),
        }
        result = self.handler.confirm_voice_program_run({"pendingRunId": "run-1", "answer": "no"})
        self.assertTrue(result["cancelled"])
        self.assertNotIn("run-1", DashboardHandler.realtime_pending_runs)
        self.handler.scene.release_plan_reservations.assert_called_once_with(plan)

    def test_stale_execution_releases_plan_reservation(self):
        plan = {"objectSnapshots": [{"objectId": "part-1"}]}
        self.handler.scene.physical_program_gate_error.return_value = None
        self.handler.scene.physical_program_warning.return_value = None
        self.handler.scene.validate_plan_object_snapshots.return_value = "part moved"
        result = self.handler.execute_validated_plan(plan, "RUN_PHYSICAL_PICK")
        self.assertFalse(result["ok"])
        self.assertTrue(result["staleObjectPreview"])
        self.handler.scene.release_plan_reservations.assert_called_once_with(plan)
        self.handler.service.execute_pick_plan.assert_not_called()


class ManualJointCommandSafetyTests(unittest.TestCase):
    def setUp(self):
        self.service = RobotService(None, 115200, 0.8)
        self.robot = MagicMock()
        self.service.get_robot_locked = MagicMock(return_value=self.robot)

    def test_invalid_joint_commands_never_reach_driver(self):
        for angles, speed in (
            ([0.0] * 5, 20),
            ([0.0, 0.0, 0.0, 0.0, 0.0, float("nan")], 20),
            ([169.0, 0.0, 0.0, 0.0, 0.0, 0.0], 20),
            ([0.0] * 6, 0),
            ([0.0] * 6, 101),
        ):
            self.assertFalse(self.service.send_angles(angles, speed)["ok"])
        self.robot.send_angles.assert_not_called()

    def test_valid_joint_command_reaches_driver_once(self):
        result = self.service.send_angles([0.0, -10.0, 20.0, 0.0, 0.0, -45.0], 20)
        self.assertTrue(result["ok"], result)
        self.robot.send_angles.assert_called_once_with([0.0, -10.0, 20.0, 0.0, 0.0, -45.0], 20)

    def test_unknown_command_does_not_open_or_contact_robot(self):
        result = self.service.command("not-a-command")
        self.assertFalse(result["ok"])
        self.assertIn("Unknown command", result["error"])
        self.service.get_robot_locked.assert_not_called()

    def test_driver_rejects_non_finite_robot_feedback(self):
        self.assertFalse(_valid_sextuple([0.0, 0.0, 0.0, 0.0, 0.0, float("nan")]))
        self.assertFalse(_valid_sextuple([0.0, 0.0, 0.0, 0.0, 0.0, float("inf")]))


class FrontendReliabilityTests(unittest.TestCase):
    def test_frontend_rejects_invalid_json_and_never_drags_tagged_parts(self):
        root = Path(__file__).resolve().parents[1]
        api = (root / "static/js/api.js").read_text()
        viewport = (root / "static/js/viewport.js").read_text()
        realtime = (root / "static/js/realtime.js").read_text()
        server = (root / "web_server.py").read_text()
        self.assertIn("Server returned invalid JSON", api)
        self.assertIn('selectedPart?.trackingMode === "apriltag"', viewport)
        self.assertIn("registered but its AprilTag is not visible", (root / "static/js/ui.js").read_text())
        self.assertIn("received malformed server data", realtime)
        self.assertIn('"required": ["pendingRunId", "answer"]', server)
        self.assertIn('"realtimePlanId": {"type": "string"}', server)
        self.assertIn("Default to one short sentence", server)


if __name__ == "__main__":
    unittest.main()
