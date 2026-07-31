import os
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts import run_offline_checks


ROOT = Path(__file__).resolve().parents[1]


class OfflineChecksRunnerTests(unittest.TestCase):
    def test_runner_installs_declared_dependencies_in_its_virtualenv(self):
        virtualenv_python = Path("/tmp/offline-test-venv/bin/python")
        calls = []

        def record(command, **kwargs):
            calls.append((command, kwargs))

        with patch.object(run_offline_checks, "run", side_effect=record):
            run_offline_checks.run_checks(virtualenv_python)

        self.assertEqual(
            calls[0][0],
            [
                str(virtualenv_python),
                "-m",
                "pip",
                "install",
                "--disable-pip-version-check",
                "--no-input",
                "--requirement",
                str(ROOT / "requirements.txt"),
            ],
        )
        self.assertEqual(calls[1][0], [str(virtualenv_python), "-m", "compileall", "-q", "."])
        self.assertEqual(calls[2][0], ["git", "diff", "--check"])
        self.assertEqual(
            calls[3][0],
            [str(virtualenv_python), "-m", "unittest", "discover", "-s", "tests"],
        )
        self.assertTrue(all(call[1]["env"]["PYTHONPATH"] == str(ROOT) for call in calls))

    def test_runner_selects_the_platform_virtualenv_interpreter(self):
        virtualenv = Path("/tmp/offline-test-venv")
        expected = "Scripts/python.exe" if os.name == "nt" else "bin/python"
        self.assertEqual(
            run_offline_checks.virtualenv_python(virtualenv),
            virtualenv / expected,
        )


if __name__ == "__main__":
    unittest.main()
