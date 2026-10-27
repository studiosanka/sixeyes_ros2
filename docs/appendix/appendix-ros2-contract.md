# Appendix: what was checked against ROS 2

ROS 2 and ros2_control change between releases, so this records what the code was written against and what is **unverified**.

## Target

**ROS 2 Jazzy Jalisco** (an LTS release, supported until 2029) on Ubuntu 24.04, with Gazebo Harmonic (`ros_gz`). Newer releases may need changes; check the `ros2_control` release notes first.

## Read on 2026-10-27

- The ros2_control demos, `jazzy` branch, example 2 (a complete hardware component): `diffbot_system.hpp/.cpp`, `CMakeLists.txt` and its xacro. From them: `on_init(const HardwareComponentInterfaceParams &)`, the lifecycle callbacks, `read`/`write` signatures, `set_state(name, v)` and `get_command(name)` by interface name, interfaces exported by the framework from the URDF (no `export_*` functions), `PLUGINLIB_EXPORT_CLASS(..., hardware_interface::SystemInterface)`, the CMake layout (`pluginlib_export_plugin_description_file(hardware_interface ...)`) and the `<ros2_control>` / `mock_components/GenericSystem` xacro pattern.
- The ros2_control Jazzy API pages: the `HardwareComponentInterfaceParams` overload of `on_init`, and that the old `export_*_interfaces` functions are deprecated.
- MoveIt's `MoveItConfigsBuilder` source (`jazzy`): the builder methods used, that absolute `file_path`s work, and that missing pipeline files fall back to bundled defaults (so there is no `ompl_planning.yaml` here). The MoveIt tutorial: `position_only_ik` is a KDL-plugin option.

## Verified without ROS (this repository's own checks)

- `python scripts/check.py`: names agree across the model, controllers, MoveIt files, SRDF and the arm model; the plugin description matches the class; vendored code is current; YAML and XML parse; launch files are valid Python.
- The model xacro **expands** for `mock`, `real` and `gazebo` (using the PyPI `xacro` against the real arm model) with the expected plugin, parameters, six position-only joints and the Gazebo block only for Gazebo.
- The C++ session logic is tested in `sixeyes-host` against a scripted fake arm (CI).

## Not verified

- **Compilation.** `sixeyes_hardware` and the C++ session were never compiled. Expect typos or API mismatches on the first build.
- **Launching.** None of the launch files has run. In particular: `EqualsSubstitution` (needs a recent `launch`), passing it as `use_sim_time`, and the spawner timeout while the real arm homes.
- **Gazebo.** The plugin names (`gz_ros2_control/GazeboSimSystem`, `gz_ros2_control-system`, `gz_ros2_control::GazeboSimROS2ControlPlugin`), the `gz_sim.launch.py` arguments and the clock bridge are from memory of Jazzy-era examples, not read. The placeholder model's inertias may also need tuning to stand still in physics.
- **MoveIt.** Whether the SRDF's collision list is enough for planning to start, the pose-goal behaviour with `position_only_ik`, and every launch parameter.
- **A real arm.** The firmware has not run on hardware either.

The first CI run (`.github/workflows/ci.yml`) is the first real test of all of the above.
