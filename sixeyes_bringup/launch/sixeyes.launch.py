"""Start the arm with ros2_control:  ros2 launch sixeyes_bringup sixeyes.launch.py hardware:=mock

    hardware:=mock     a pretend arm, no hardware needed (default)
    hardware:=gazebo   Gazebo simulation
    hardware:=real     the real arm over USB; also give port:=/dev/ttyACM0 (the arm homes when it starts: keep clear)
    use_rviz:=false    do not open RViz

NOT TESTED with ROS 2 yet: the first run is the CI smoke test (see docs/appendix/appendix-testing.md).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EqualsSubstitution, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = LaunchConfiguration("hardware")
    port = LaunchConfiguration("port")
    use_rviz = LaunchConfiguration("use_rviz")
    is_gazebo = EqualsSubstitution(hardware, "gazebo")

    controllers = PathJoinSubstitution([FindPackageShare("sixeyes_bringup"), "config", "controllers.yaml"])
    model = PathJoinSubstitution([FindPackageShare("sixeyes_bringup"), "urdf", "sixeyes_system.urdf.xacro"])
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model, " hardware:=", hardware, " port:=", port, " controllers_file:=", controllers]),
        value_type=str,
    )
    sim_time = {"use_sim_time": is_gazebo}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}, sim_time],
        output="screen",
    )

    # With mock or real hardware we run the controller manager ourselves. In Gazebo the Gazebo plugin runs it.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers],
        output="screen",
        condition=UnlessCondition(is_gazebo),
    )

    # Homing the real arm happens while the hardware activates, which can take a minute: wait for it.
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "180"],
            output="screen",
        )
        for name in ("joint_state_broadcaster", "arm_controller", "gripper_controller")
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
        condition=IfCondition(is_gazebo),
    )
    spawn_in_gazebo = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "sixeyes"],
        output="screen",
        condition=IfCondition(is_gazebo),
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        condition=IfCondition(is_gazebo),
    )

    rviz = Node(package="rviz2", executable="rviz2", output="log", parameters=[sim_time], condition=IfCondition(use_rviz))

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware", default_value="mock", choices=["mock", "gazebo", "real"], description="Which arm to drive"),
            DeclareLaunchArgument("port", default_value="", description="USB port of the real arm, e.g. /dev/ttyACM0"),
            DeclareLaunchArgument("use_rviz", default_value="true", description="Open RViz"),
            robot_state_publisher,
            control_node,
            gazebo,
            spawn_in_gazebo,
            clock_bridge,
            *spawners,
            rviz,
        ]
    )
