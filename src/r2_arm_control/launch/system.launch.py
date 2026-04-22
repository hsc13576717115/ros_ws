import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    control_share = get_package_share_directory("r2_arm_control")
    moveit_launch = os.path.join(
        get_package_share_directory("r2_arm_moveit_config"),
        "launch",
        "moveit.launch.py",
    )

    motors_yaml = LaunchConfiguration("motors_yaml")
    arm_yaml = LaunchConfiguration("arm_yaml")
    start_moveit = LaunchConfiguration("start_moveit")
    start_state_machine = LaunchConfiguration("start_state_machine")
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    arm_config = _load_yaml(arm_yaml.perform(context))

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                "motors_yaml": motors_yaml,
                "arm_yaml": arm_yaml,
                "start_rviz": start_rviz,
                "use_sim_time": use_sim_time,
            }.items(),
            condition=IfCondition(start_moveit),
        ),
        Node(
            package="r2_arm_control",
            executable="arm_state_machine_node.py",
            output="screen",
            parameters=[
                arm_config,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(start_state_machine),
        ),
    ]


def generate_launch_description():
    control_share = get_package_share_directory("r2_arm_control")
    default_motors_yaml = os.path.join(control_share, "config", "motors.yaml")
    default_arm_yaml = os.path.join(control_share, "config", "arm.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument("start_state_machine", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("motors_yaml", default_value=default_motors_yaml),
            DeclareLaunchArgument("arm_yaml", default_value=default_arm_yaml),
            OpaqueFunction(function=_launch_setup),
        ]
    )
