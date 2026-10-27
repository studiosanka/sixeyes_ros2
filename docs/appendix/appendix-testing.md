# Appendix: testing

## What can be checked without ROS

```
python scripts/check.py          # names, files, vendored code, YAML/XML/launch syntax
```

Needs `sixeyes_description` and `sixeyes-host` next to this repository, and PyYAML. It prints what it could not check. When a deliberate mistake was put in a copy (a misspelled joint, a wrong plugin class, an edited vendored file, a bad SRDF link) it caught all four.

## CI (`.github/workflows/ci.yml`), not run yet

| Job | What |
|---|---|
| `static` | `scripts/check.py` |
| `ros` | In a `ros:jazzy` container: `rosdep install`, `colcon build` of all packages, expand the model for `mock`, `real` and `gazebo` and run `check_urdf` on each, then the smoke test |
| smoke test | `scripts/smoke_test.sh`: launch `hardware:=mock` without RViz, wait for all three controllers to be active, send a small trajectory, and check `/joint_states` follows it |

## Checklist for the first real run

Only after the `ros` job is green.

1. `hardware:=mock` on a real ROS 2 install: RViz shows the arm; MoveIt plans to the `ready` and `home` states and the arm in RViz follows.
2. `hardware:=gazebo`: the arm spawns and stands still (fix inertias or add damping if it collapses), and the controllers go active.
3. **Real arm, free joints, nothing attached to the gearbox:** `hardware:=real port:=...`. The arm homes (keep clear). Send a very small joint-space move and check each joint moves the right way and the step limit holds.
4. Pull the USB cable while it moves: the arm must stop within a fraction of a second and the launch must report "No recent data from the arm".
5. Press the BOOT stop button on any board: the arm stops, the component reports the stop, and the controllers deactivate.
6. Run MoveIt plans on the real arm at low speed scaling, with someone at the power switch.
