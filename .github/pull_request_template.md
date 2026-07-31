## Summary

Describe the operator-visible change and why it is needed.

## Reproduction or design evidence

Link the issue and include the smallest reproducible case.

## Safety impact

- [ ] No physical-motion behavior changed
- [ ] Motion behavior changed and the safety implications are explained below
- [ ] No validation or confirmation gate was weakened
- [ ] No robot or camera was accessed by automated tests

## Data compatibility

- [ ] Existing workcell data remains compatible
- [ ] Migration behavior is documented and tested
- [ ] No personal `data/workcell.json`, secret, or device identifier is included

## Validation

- [ ] Regression tests added or updated
- [ ] `python3 scripts/run_offline_checks.py`
- [ ] `git diff --check`
- [ ] Physical testing still required is listed below

## Physical validation still required

State `None` only when the change cannot affect physical behavior.
