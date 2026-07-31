# Contributing

Thank you for helping improve the myCobot 280 Spatial Workcell.

Read the [Safety](README.md#safety), [Testing](README.md#testing), and [Automated repository maintenance](README.md#automated-repository-maintenance) sections before making a change.

## Before opening an issue

- Search existing issues.
- Reproduce the problem on the latest default branch.
- Remove API keys, device identifiers, and personal workcell data from logs.
- State whether any physical motion occurred.

Use the provided bug or feature issue form.

## Development setup

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
PYTHONPATH=. python3 -m unittest discover -s tests
```

## Pull requests

1. Create a focused branch.
2. Preserve backward compatibility or include an explicit migration.
3. Add a regression test for bug fixes.
4. Run the complete offline suite.
5. Run `git diff --check`.
6. Explain safety impact and any operator-controlled physical testing still required.
7. Do not include `data/workcell.json`, `.env`, `api_keys.env`, camera IDs, serial IDs, or personal calibration.

Automated testing must never connect to robot or camera hardware.

## Review principles

- Reject unavailable coordinates instead of inventing them.
- Keep robot-base, flange, TCP, and contact frames explicit.
- Keep computer vision classification separate from geometry.
- Keep deterministic server code authoritative for coordinates.
- Preserve explicit confirmation and feedback verification for physical motion.

## License

The maintainer must add a root open-source license before outside contributions can be accepted under clear reuse terms.
