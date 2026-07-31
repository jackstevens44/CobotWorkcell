#!/usr/bin/env python3
"""Run the complete software-only validation suite in an isolated environment."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Sequence


ROOT = Path(__file__).resolve().parents[1]
REQUIREMENTS = ROOT / "requirements.txt"
MINIMUM_PYTHON = (3, 10)


def run(command: Sequence[str], *, env: dict[str, str] | None = None) -> None:
    """Run one fixed validation command from the repository root."""
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def require_supported_python() -> None:
    if sys.version_info < MINIMUM_PYTHON:
        required = ".".join(str(part) for part in MINIMUM_PYTHON)
        actual = ".".join(str(part) for part in sys.version_info[:3])
        raise SystemExit(
            f"Python {required} or newer is required; this interpreter is {actual}. "
            "Run this script with python3."
        )


def virtualenv_python(virtualenv: Path) -> Path:
    if os.name == "nt":
        return virtualenv / "Scripts" / "python.exe"
    return virtualenv / "bin" / "python"


def run_checks(python: Path) -> None:
    """Install declared dependencies, then run only static and offline checks."""
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(ROOT)
    run(
        [
            str(python),
            "-m",
            "pip",
            "install",
            "--disable-pip-version-check",
            "--no-input",
            "--requirement",
            str(REQUIREMENTS),
        ],
        env=environment,
    )
    run([str(python), "-m", "compileall", "-q", "."], env=environment)
    run(["git", "diff", "--check"], env=environment)
    run(
        [str(python), "-m", "unittest", "discover", "-s", "tests"],
        env=environment,
    )


def main() -> None:
    require_supported_python()
    with tempfile.TemporaryDirectory(prefix="cobotworkcell-offline-tests-") as temporary:
        virtualenv = Path(temporary) / "venv"
        run([sys.executable, "-m", "venv", str(virtualenv)])
        run_checks(virtualenv_python(virtualenv))


if __name__ == "__main__":
    main()
