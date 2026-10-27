# Appendix: how it works

## The three packages

| Package | What |
|---|---|
| `sixeyes_hardware` | A `ros2_control` hardware component (`sixeyes_hardware/SixEyesSystem`) that talks to the arm over USB |
| `sixeyes_bringup` | The model with its `ros2_control` setup (one xacro, three hardware choices), the controller settings, and the launch file |
| `sixeyes_moveit_config` | MoveIt: the SRDF (groups, poses, collision list), the IK settings, how MoveIt sends movements to the controllers |

The arm's shape comes from `sixeyes_description` (a separate repository), including its placeholder joint limits.

## One model, three hardware choices

`sixeyes_bringup/urdf/sixeyes_system.urdf.xacro` includes the arm model, adds a fixed `world` frame, and adds a `ros2_control` block whose `<plugin>` depends on `hardware:=`:

- `mock` -> `mock_components/GenericSystem` (ros2_control's pretend hardware)
- `real` -> `sixeyes_hardware/SixEyesSystem`, with `port`
- `gazebo` -> `gz_ros2_control/GazeboSimSystem`, plus the Gazebo plugin that runs the controllers

Everything else (the six joints with one `position` command and state each, the controllers, MoveIt) is identical.

## The hardware component

- **Joints, by name:** `base, shoulder, elbow, wrist_pitch, wrist_roll, gripper`. The five angles are radians; the **gripper is metres** of finger travel in ROS (the model's prismatic joint), scaled to and from the arm's 0..100 with `gripper_stroke_m`.
- **Lifecycle:** `on_configure` opens the USB port, says hello (checks the protocol version and that the firmware can home), and takes control (`set_control_source usb`). `on_activate` homes the arm (unless `home_on_activate=false`) and starts by holding where the arm is, so activating never makes it jump. `on_deactivate` sends Stop.
- **Each cycle:** `read()` copies the arm's measured joints into the state interfaces; no data for 0.5 s is an error. `write()` sends the commanded positions, **limited to `max_step_rad` (0.35) from where each joint is now**, whatever the controller asks. Nothing is sent until the arm is homed.
- **Faults:** an e-stop or a lost joint board makes `read()` return an error, which stops the controllers. After a fault, fix it and restart the launch (the component does not re-home by itself).
- **Heartbeat:** the C++ session sends one every 50 ms while the arm is ready and under USB control, so the arm's 200 ms watchdog stays fed. If the program dies, the arm stops.
- The C++ session code is **copied** from the `sixeyes-host` repository (`python scripts/sync-host-core.py`), so this repository builds on its own. Never edit the copies.

## Controllers and MoveIt

`arm_controller` (five joints) and `gripper_controller` (one) are `JointTrajectoryController`s with position command and state interfaces only (the arm reports no velocities, so `allow_nonzero_velocity_at_trajectory_end` is on). MoveIt talks to them through `FollowJointTrajectory`.

The arm has **5 joints**, so most full poses (position and orientation) are unreachable. `kinematics.yaml` therefore sets `position_only_ik: true`: MoveIt solves for the tool's position and ignores its orientation. Joint-space goals can still set every angle.

## After the CAD is final

1. Regenerate the model in `sixeyes_description` (see its docs). Joint limits flow into MoveIt automatically.
2. Redo the collision list: the SRDF's `disable_collisions` lines are for the placeholder shapes. Run the MoveIt Setup Assistant on the new model and copy its list in.
3. Check the `ready` pose in the SRDF is a sensible, collision-free pose for the real arm.
