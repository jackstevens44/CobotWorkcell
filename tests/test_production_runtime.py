import tempfile
import threading
import time
import unittest
from copy import deepcopy
from pathlib import Path

from web_server import ProductionProgramRuntime
from workcell import HOME_ANGLES, PHYSICAL_CONFIRM_TOKEN, Workcell


class SupportSurfaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cell = Workcell(Path(self.temp.name))

    def tearDown(self):
        self.temp.cleanup()

    def test_main_table_is_automatic_and_raised_surface_persists(self):
        table = self.cell.support_surfaces["surface-table"]
        self.assertEqual(table["topZ"], 0.0)
        self.assertTrue(table["locked"])
        saved = self.cell.upsert_support_surface({
            "name": "Shelf", "center": {"x": 0.2, "y": 0.0},
            "size": {"x": 0.18, "y": 0.12}, "topZ": 0.09,
            "entryToleranceM": 0.015, "holdToleranceM": 0.02,
        })
        surface_id = saved["supportSurface"]["id"]
        reloaded = Workcell(Path(self.temp.name))
        self.assertAlmostEqual(reloaded.support_surfaces[surface_id]["topZ"], 0.09)
        self.assertFalse(reloaded.delete_support_surface("surface-table")["ok"])

    def test_gripper_clearance_is_relative_to_matched_surface(self):
        part = {
            "position": {"x": 0.2, "y": 0.0, "z": 0.12},
            "size": {"x": 0.05, "y": 0.04, "z": 0.04},
            "supportSurfaceZ": 0.10,
        }
        model = self.cell._grasp_height_model(part)
        self.assertAlmostEqual(model["objectBottomZ"], 0.10)
        self.assertAlmostEqual(model["supportSurfaceZ"], 0.10)
        self.assertAlmostEqual(model["tableClearanceM"], self.cell._minimum_table_clearance_m())

    def test_raised_surface_intersection_increases_transfer_clearance(self):
        self.cell.upsert_support_surface({
            "id": "platform", "name": "Platform", "center": {"x": 0.2, "y": 0.0},
            "size": {"x": 0.10, "y": 0.12}, "topZ": 0.12,
        })
        clearance = self.cell._surface_transfer_clearance_z(
            (0.10, 0.0), (0.30, 0.0), carried_depth_below_tcp=0.04,
        )
        self.assertGreaterEqual(clearance, 0.18)


class ProgramCacheTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cell = Workcell(Path(self.temp.name))

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def valid_plan():
        return {
            "ok": True, "mode": "coordinate_program", "program": "Cycle",
            "physicalReady": True, "coordinatePreview": {"ok": True, "states": []},
            "steps": [{"stateId": "home", "robotCommand": "home", "sourceStepId": "home-1"}],
            "objectSnapshots": [], "destinationSnapshots": [], "durationMs": 100,
            "motionModel": {}, "safetyGate": {},
        }

    def test_legacy_repeat_migrates_to_finite_run_policy(self):
        program = self.cell.normalized_program({
            "name": "Legacy", "repeatCount": 4,
            "steps": [{"type": "home"}],
        })
        self.assertEqual(program["runPolicy"]["mode"], "finite")
        self.assertEqual(program["runPolicy"]["cycleCount"], 4)

    def test_validated_cycle_persists_and_surface_change_marks_it_stale(self):
        saved = self.cell.save_program({
            "editorVersion": 2, "name": "Cycle",
            "runPolicy": {"mode": "continuous", "maxCycles": 2},
            "steps": [{"id": "home-1", "type": "home"}],
        })["program"]
        cached = self.cell.persist_compiled_cycle(saved["id"], self.valid_plan(), HOME_ANGLES)
        self.assertTrue(cached["ok"], cached)
        self.assertIsNone(self.cell.compiled_cycle_error(self.cell.programs[saved["id"]]))
        self.cell.upsert_support_surface({
            "name": "Platform", "center": {"x": 0.2, "y": 0},
            "size": {"x": 0.1, "y": 0.1}, "topZ": 0.05,
        })
        self.assertIn("stale", self.cell.compiled_cycle_error(self.cell.programs[saved["id"]]))

    def test_failed_compile_does_not_replace_last_valid_cycle(self):
        saved = self.cell.save_program({
            "editorVersion": 2, "name": "Cycle",
            "steps": [{"id": "home-1", "type": "home"}],
        })["program"]
        cached = self.cell.persist_compiled_cycle(saved["id"], self.valid_plan(), HOME_ANGLES)
        generated_at = cached["compiledCycle"]["generatedAt"]
        failed = self.cell.persist_compiled_cycle(
            saved["id"], {"ok": False, "coordinatePreview": {"ok": False}}, HOME_ANGLES,
        )
        self.assertFalse(failed["ok"])
        self.assertEqual(
            self.cell.programs[saved["id"]]["compiledCycle"]["generatedAt"], generated_at,
        )

    def test_simulation_only_destination_keeps_validated_preview_and_exact_blocker(self):
        saved = self.cell.save_program({
            "editorVersion": 2, "name": "Cycle",
            "steps": [{"id": "home-1", "type": "home"}],
        })["program"]
        plan = self.valid_plan()
        plan["physicalReady"] = False
        plan["unverifiedDestinations"] = [{
            "kind": "bin", "id": "bin-a", "label": "Bin A",
            "positionStatus": "simulation_only",
        }]
        cached = self.cell.persist_compiled_cycle(saved["id"], plan, HOME_ANGLES)
        self.assertTrue(cached["ok"], cached)
        self.assertEqual(cached["compiledCycle"]["status"], "validated")
        self.assertFalse(cached["compiledCycle"]["planTemplate"]["physicalReady"])
        blocker = self.cell.compiled_cycle_error(self.cell.programs[saved["id"]])
        self.assertIn("Bin A is simulation-only", blocker)
        self.assertIn("confirm its physical position", blocker)


class FakeProductionService:
    def __init__(self):
        self.executions = 0
        self.started = threading.Event()
        self.release = threading.Event()

    def status(self):
        return {"ok": True, "lastAngles": list(HOME_ANGLES)}

    def set_end_effector(self, _tool):
        pass

    def set_tool_profile(self, _profile):
        pass

    def add_coordinate_preview(self, plan, _angles):
        result = deepcopy(plan)
        result["ok"] = True
        result["physicalReady"] = True
        result["coordinatePreview"] = {"ok": True, "states": []}
        return result

    def execute_pick_plan(self, plan, confirm):
        self.started.set()
        self.release.wait(0.25)
        self.executions += 1
        return {"ok": confirm == PHYSICAL_CONFIRM_TOKEN, "executedSteps": []}

    def command(self, _command):
        self.release.set()
        return {"ok": True}


class ProductionRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cell = Workcell(Path(self.temp.name))
        self.service = FakeProductionService()

    def tearDown(self):
        self.service.release.set()
        if hasattr(self, "runtime"):
            self.runtime.shutdown()
        self.temp.cleanup()

    def save_cached_program(self, mode, maximum=None):
        program = self.cell.save_program({
            "editorVersion": 2, "name": "Production cycle",
            "runPolicy": {"mode": mode, "maxCycles": maximum},
            "steps": [{"id": "home-1", "type": "home"}],
        })["program"]
        plan = ProgramCacheTests.valid_plan()
        self.cell.persist_compiled_cycle(program["id"], plan, HOME_ANGLES)
        return self.cell.programs[program["id"]]

    def wait_for(self, predicate, timeout=2.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def test_continuous_program_uses_one_confirmation_and_stops_at_maximum(self):
        program = self.save_cached_program("continuous", maximum=2)
        self.runtime = ProductionProgramRuntime(self.cell, self.service)
        self.service.release.set()
        armed = self.runtime.arm(program["id"], PHYSICAL_CONFIRM_TOKEN, 25)
        self.assertTrue(armed["ok"], armed)
        self.assertTrue(self.wait_for(lambda: self.runtime.status()["state"] == "completed"))
        self.assertEqual(self.runtime.status()["cycleCount"], 2)
        self.assertEqual(self.service.executions, 2)

    def test_external_triggers_are_coalesced_to_one_pending_cycle(self):
        program = self.save_cached_program("external_triggered")
        self.runtime = ProductionProgramRuntime(self.cell, self.service)
        armed = self.runtime.arm(program["id"], PHYSICAL_CONFIRM_TOKEN)
        self.assertTrue(armed["ok"])
        self.runtime.trigger()
        self.assertTrue(self.service.started.wait(1.0))
        self.runtime.trigger()
        self.runtime.trigger()
        self.service.release.set()
        self.assertTrue(self.wait_for(lambda: self.service.executions >= 2))
        time.sleep(0.15)
        self.assertEqual(self.service.executions, 2)
        self.runtime.stop()

    def test_stop_disarms_and_bad_confirmation_never_arms(self):
        program = self.save_cached_program("continuous")
        self.runtime = ProductionProgramRuntime(self.cell, self.service)
        self.assertFalse(self.runtime.arm(program["id"], "wrong")["ok"])
        self.assertTrue(self.runtime.arm(program["id"], PHYSICAL_CONFIRM_TOKEN)["ok"])
        self.runtime.stop()
        self.assertEqual(self.runtime.status()["state"], "disarmed")

    def test_validated_simulation_with_unconfirmed_bin_cannot_arm_hardware(self):
        program = self.cell.save_program({
            "editorVersion": 2, "name": "Blocked cycle",
            "steps": [{"id": "home-1", "type": "home"}],
        })["program"]
        plan = ProgramCacheTests.valid_plan()
        plan["physicalReady"] = False
        plan["unverifiedDestinations"] = [{
            "kind": "bin", "id": "bin-a", "label": "Bin A",
            "positionStatus": "simulation_only",
        }]
        self.cell.persist_compiled_cycle(program["id"], plan, HOME_ANGLES)
        self.runtime = ProductionProgramRuntime(self.cell, self.service)
        result = self.runtime.arm(program["id"], PHYSICAL_CONFIRM_TOKEN)
        self.assertFalse(result["ok"])
        self.assertIn("Bin A is simulation-only", result["error"])
        self.assertEqual(self.service.executions, 0)

    def test_object_trigger_requires_stable_visibility_and_then_waits_for_removal(self):
        definition = self.cell.bind_tagged_part({
            "partId": "trigger-part", "tagId": 10, "label": "Incoming Part",
            "size": {"x": 0.04, "y": 0.04, "z": 0.03},
        })["registeredPart"]
        program = self.cell.save_program({
            "editorVersion": 2, "name": "Arrival cycle",
            "runPolicy": {
                "mode": "object_triggered", "triggerPartId": definition["partId"],
                "expectedSurfaceId": "surface-table", "cooldownMs": 0,
            },
            "steps": [{"id": "home-1", "type": "home"}],
        })["program"]
        self.cell.persist_compiled_cycle(program["id"], ProgramCacheTests.valid_plan(), HOME_ANGLES)
        self.cell.programs[program["id"]]["compiledCycle"]["triggerAnchor"] = {
            "objectId": definition["partId"],
            "position": {"x": 0.20, "y": 0.05, "z": 0.015},
            "orientationDeg": 0.0,
            "supportSurfaceId": "surface-table", "supportSurfaceZ": 0.0,
        }
        self.cell.ingest_tag_tracks([{
            "id": definition["partId"], "localizationSource": "object_tag",
            "position": {"x": 0.20, "y": 0.05, "z": 0.015},
            "orientationDeg": 0.0, "supportSurfaceId": "surface-table",
            "supportSurfaceName": "Main Table", "supportSurfaceZ": 0.0,
        }], valid=True)
        self.runtime = ProductionProgramRuntime(self.cell, self.service)
        self.service.release.set()
        armed = self.runtime.arm(program["id"], PHYSICAL_CONFIRM_TOKEN)
        self.assertTrue(armed["ok"], armed)
        for timestamp in (time.time() + 0.01, time.time() + 0.02):
            time.sleep(0.12)
            self.cell.ingest_tag_tracks([{
                "id": definition["partId"], "localizationSource": "object_tag",
                "position": {"x": 0.20, "y": 0.05, "z": 0.015},
                "orientationDeg": 0.0, "supportSurfaceId": "surface-table",
                "supportSurfaceName": "Main Table", "supportSurfaceZ": 0.0,
            }], timestamp=timestamp, valid=True)
        self.assertTrue(self.wait_for(lambda: self.service.executions == 1))
        time.sleep(0.35)
        self.assertEqual(self.service.executions, 1)
        self.assertEqual(self.runtime.status()["cycleCount"], 1)

    def test_object_trigger_outside_cached_envelope_faults_before_execution(self):
        definition = self.cell.bind_tagged_part({
            "partId": "trigger-part", "tagId": 10, "label": "Incoming Part",
            "size": {"x": 0.04, "y": 0.04, "z": 0.03},
        })["registeredPart"]
        program = self.cell.save_program({
            "editorVersion": 2, "name": "Arrival cycle",
            "runPolicy": {
                "mode": "object_triggered", "triggerPartId": definition["partId"],
                "expectedSurfaceId": "surface-table", "cooldownMs": 0,
                "xyEnvelopeM": 0.015,
            },
            "steps": [{"id": "home-1", "type": "home"}],
        })["program"]
        self.cell.persist_compiled_cycle(program["id"], ProgramCacheTests.valid_plan(), HOME_ANGLES)
        self.cell.programs[program["id"]]["compiledCycle"]["triggerAnchor"] = {
            "objectId": definition["partId"],
            "position": {"x": 0.20, "y": 0.05, "z": 0.015},
            "orientationDeg": 0.0,
            "supportSurfaceId": "surface-table", "supportSurfaceZ": 0.0,
        }
        self.cell.ingest_tag_tracks([{
            "id": definition["partId"], "localizationSource": "object_tag",
            "position": {"x": 0.22, "y": 0.05, "z": 0.015},
            "orientationDeg": 0.0, "supportSurfaceId": "surface-table",
            "supportSurfaceName": "Main Table", "supportSurfaceZ": 0.0,
        }], valid=True)
        self.runtime = ProductionProgramRuntime(self.cell, self.service)
        self.service.release.set()
        armed = self.runtime.arm(program["id"], PHYSICAL_CONFIRM_TOKEN)
        self.assertTrue(armed["ok"], armed)
        for timestamp in (time.time() + 0.01, time.time() + 0.02):
            time.sleep(0.12)
            self.cell.ingest_tag_tracks([{
                "id": definition["partId"], "localizationSource": "object_tag",
                "position": {"x": 0.22, "y": 0.05, "z": 0.015},
                "orientationDeg": 0.0, "supportSurfaceId": "surface-table",
                "supportSurfaceName": "Main Table", "supportSurfaceZ": 0.0,
            }], timestamp=timestamp, valid=True)
        self.assertTrue(self.wait_for(lambda: self.runtime.status()["state"] == "faulted"))
        self.assertIn("outside", self.runtime.status()["lastError"])
        self.assertEqual(self.service.executions, 0)


class ProductionFrontendContractTests(unittest.TestCase):
    def test_support_surface_and_run_mode_controls_are_wired(self):
        root = Path(__file__).resolve().parents[1]
        html = (root / "static/index.html").read_text()
        ui = (root / "static/js/ui.js").read_text()
        viewport = (root / "static/js/viewport.js").read_text()
        server = (root / "web_server.py").read_text()
        for control in (
            'id="supportSurfaceDialog"', 'id="programRunMode"',
            'id="programTriggerPart"', 'id="triggerCycleBtn"',
        ):
            self.assertIn(control, html)
        self.assertIn('/api/scene/support-surface', ui)
        self.assertIn('/api/program/runtime/arm', ui)
        self.assertIn('/api/program/runtime/trigger', ui)
        self.assertIn("function makeSupportSurfaceMesh(surface)", viewport)
        self.assertIn("for (const surface of state.supportSurfaces || [])", viewport)
        self.assertIn('if parsed.path == "/api/program/runtime/status"', server)


if __name__ == "__main__":
    unittest.main()
