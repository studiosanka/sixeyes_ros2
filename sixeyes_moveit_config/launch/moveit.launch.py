"""Plan and move the arm with MoveIt. Start the arm first (sixeyes_bringup), then in another terminal:

    ros2 launch sixeyes_moveit_config moveit.launch.py hardware:=mock

Give the same hardware:= as you gave sixeyes_bringup (it only changes how the model is built).
NOT TESTED with ROS 2 yet: the first run is the CI smoke test (see docs/appendix/appendix-testing.md).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    hardware = LaunchConfiguration("hardware").perform(context)
    port = LaunchConfiguration("port").perform(context)
    use_sim_time = hardware == "gazebo"

    model = os.path.join(get_package_share_directory("sixeyes_bringup"), "urdf", "sixeyes_system.urdf.xacro")
    # The arm's speed and acceleration limits come from the arm model (placeholders until the CAD is final).
    limits = os.path.join(get_package_share_directory("sixeyes_description"), "config", "joint_limits.yaml")

    moveit_config = (
        MoveItConfigsBuilder("sixeyes", package_name="sixeyes_moveit_config")
        .robot_description(file_path=model, mappings={"hardware": hardware, "port": port})
        .robot_description_semantic(file_path="config/sixeyes.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path=limits)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware", default_value="mock", choices=["mock", "gazebo", "real"]),
            DeclareLaunchArgument("port", default_value="", description="USB port, only used to build the model"),
            OpaqueFunction(function=launch_setup),
        ]
    )
