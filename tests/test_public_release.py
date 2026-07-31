import re
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class PublicReleaseTests(unittest.TestCase):
    def test_tracked_tree_has_no_private_runtime_files(self):
        tracked = subprocess.run(
            ["git", "ls-files"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.splitlines()
        forbidden_names = {".env", "api_keys.env", "workcell.json", ".DS_Store"}
        self.assertFalse(
            [path for path in tracked if Path(path).name in forbidden_names],
            "Private runtime data or OS metadata must not be tracked.",
        )

    def test_public_text_has_no_personal_paths_or_concrete_macos_serial_ids(self):
        source_suffixes = {".py", ".sh", ".md", ".yml", ".yaml", ".json", ".txt"}
        concrete_serial = re.compile(
            r"/dev/cu\." + r"usbserial-(?!X{4,}\b)[A-Za-z0-9]+"
        )
        personal_home_prefix = "/" + "Users/"
        tracked = subprocess.run(
            ["git", "ls-files"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.splitlines()
        violations = []
        for relative_path in tracked:
            path = ROOT / relative_path
            if not path.is_file() or path.suffix.lower() not in source_suffixes:
                continue
            text = path.read_text(encoding="utf-8", errors="replace")
            if personal_home_prefix in text or concrete_serial.search(text):
                violations.append(str(path.relative_to(ROOT)))
        self.assertFalse(
            violations,
            f"Machine-specific public values found in: {', '.join(violations)}",
        )

    def test_restart_script_defaults_to_disconnected_portable_startup(self):
        script = (ROOT / "restart_server.sh").read_text(encoding="utf-8")
        self.assertIn('PORT="${ROBOT_PORT:-}"', script)
        self.assertIn('PY="${PYTHON_BIN:-python3}"', script)
        self.assertIn('if [ -n "$PORT" ]; then', script)
        self.assertNotIn('--port "$PORT" --baud', script)


if __name__ == "__main__":
    unittest.main()
