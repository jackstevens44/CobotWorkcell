# Repository automation rules

These rules apply to human-assisted and automated coding agents.

## Safety boundary

- Never connect to a serial port, camera, gripper, pump, or physical robot during automated work.
- Never issue joint, Cartesian, jog, IO, suction, gripper, or hand-guide commands.
- Use fakes and the offline test suite for all automated validation.
- Do not weaken kinematic, calibration, freshness, collision, confirmation, or motion-verification checks merely to make a test pass.

## Data and secrets

- Do not modify or commit the operator's live `data/workcell.json`.
- Do not read, print, copy, or commit `.env`, `api_keys.env`, tokens, serial identifiers, or camera identifiers.
- Use examples and synthetic fixtures in tests.

## Change policy

- Preserve unrelated worktree changes.
- Add a regression test for every reproducible bug fix.
- Run `PYTHONPATH=. python -m unittest discover -s tests` and `git diff --check`.
- State what was tested offline and what still requires operator-controlled physical validation.
- Automated changes must use a dedicated branch and pull request.
- Never push directly to `main`, merge a pull request, tag a version, or publish a release without maintainer approval.

## Release policy

Agents may recommend a semantic version and draft release notes. The maintainer decides when to merge, tag, and publish.
