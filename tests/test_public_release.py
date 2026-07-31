import re
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class PublicReleaseTests(unittest.TestCase):
    def test_root_apache_license_and_notice_are_present(self):
        license_text = (ROOT / "LICENSE").read_text(encoding="utf-8")
        notice = (ROOT / "NOTICE").read_text(encoding="utf-8")
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn("Apache License", license_text)
        self.assertIn("Version 2.0, January 2004", license_text)
        self.assertIn("END OF TERMS AND CONDITIONS", license_text)
        self.assertIn("Copyright 2026 Jack Stevens", notice)
        self.assertIn("[Apache License 2.0](LICENSE)", readme)
        self.assertIn("third-party components retain", readme)
        self.assertNotIn("No root project license", readme)

    def test_public_branding_identifies_project_and_supported_robot(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        dashboard = (ROOT / "static" / "index.html").read_text(encoding="utf-8")
        self.assertTrue(readme.startswith("# CobotWorkcell\n"))
        self.assertIn("Elephant Robotics myCobot 280 M5", readme)
        self.assertIn("<h1>CobotWorkcell</h1>", dashboard)
        self.assertNotIn("AI-Workcell-M5-Cobot-Stack", readme)

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

    def test_github_actions_are_pinned_to_immutable_shas(self):
        workflow_root = ROOT / ".github" / "workflows"
        unpinned = []
        for path in workflow_root.glob("*.yml"):
            for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), start=1
            ):
                match = re.search(r"^\s*uses:\s*[^@\s]+@([^\s#]+)", line)
                if match and not re.fullmatch(r"[0-9a-f]{40}", match.group(1)):
                    unpinned.append(f"{path.name}:{line_number}")
        self.assertFalse(
            unpinned,
            f"GitHub Actions must use immutable commit SHAs: {', '.join(unpinned)}",
        )

    def test_every_vendor_group_has_a_local_license_notice(self):
        vendor_root = ROOT / "static" / "vendor"
        missing = []
        for directory in sorted(path for path in vendor_root.iterdir() if path.is_dir()):
            if not any(path.name.startswith("LICENSE") for path in directory.iterdir()):
                missing.append(directory.name)
        self.assertFalse(
            missing,
            f"Vendored components require local license notices: {', '.join(missing)}",
        )


if __name__ == "__main__":
    unittest.main()
