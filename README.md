# sixeyes_ros2

Control a [SixEyes](https://github.com/studiosanka/sixeyes) robot arm with [ROS 2](https://docs.ros.org) and [MoveIt](https://moveit.ai): plan movements, try them in a pretend arm or in Gazebo first, then run the same thing on the real arm.

> **Untested.** Written carefully against the ROS 2 Jazzy documentation, but **never built or run with ROS 2** (the machine it was written on has no ROS or C++ compiler). The first CI run is the first real test. Do not run it on a real arm until the CI is green and you have tried `hardware:=mock`.

## Quick start (pretend arm, no hardware)

You need ROS 2 **Jazzy** (Ubuntu 24.04), `sixeyes_description` next to this repository, and a workspace:

```
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/studiosanka/sixeyes_description.git
git clone https://github.com/studiosanka/sixeyes_ros2.git
cd ~/ws && rosdep install --from-paths src --ignore-src -y && colcon build
source install/setup.bash

ros2 launch sixeyes_bringup sixeyes.launch.py hardware:=mock      # the arm in RViz
ros2 launch sixeyes_moveit_config moveit.launch.py hardware:=mock # in a second terminal: plan with MoveIt
```

## Three ways to run it

| `hardware:=` | What moves | Needs |
|---|---|---|
| `mock` | A pretend arm | Nothing |
| `gazebo` | The arm in a Gazebo simulation | Gazebo (`ros_gz`) |
| `real` and `port:=/dev/ttyACM0` | **The real arm**, over USB. It **homes itself first: keep clear** | The arm, with firmware that speaks the host protocol |

Only the hardware changes between them. The controllers and the MoveIt setup are the same, so what works in `mock` and Gazebo is what will run on the arm.

## Status

| Part | State |
|---|---|
| Hardware plugin (`sixeyes_hardware`) | 🚧 written, never built |
| Launch files, controllers (`sixeyes_bringup`) | 🚧 written; the model expands correctly for all three modes (checked with `xacro`), never launched |
| MoveIt configuration | 🚧 written, never run; placeholder arm shape |
| Static consistency checks (`python scripts/check.py`) | ✅ passing; they catch mismatched names, not ROS errors |
| Gazebo | ⬜ untested, least certain of all |
| A real arm | ⬜ firmware is written but has not run on hardware |

## More

[How it works](docs/appendix/appendix-how-it-works.md) · [What was checked against ROS 2](docs/appendix/appendix-ros2-contract.md) · [Testing](docs/appendix/appendix-testing.md) · [Contributing](CONTRIBUTING.md) · Licence: Apache-2.0

Built by [Studio Sanka](https://github.com/studiosanka). Developed with AI assistance from [Claude](https://www.anthropic.com/claude) (Anthropic); the designs, decisions and review are by the Studio Sanka team.
