#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    nav2_config_dir = get_package_share_directory("nav2_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("cmd_invert_linear_x", default_value="true"),
            DeclareLaunchArgument("cmd_invert_angular_z", default_value="false"),
            DeclareLaunchArgument("start_preset_mission", default_value="false"),
            DeclareLaunchArgument("start_yolo", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument(
                "nav_start_rviz",
                default_value=LaunchConfiguration("start_rviz"),
            ),
            DeclareLaunchArgument("yolo_show_detection", default_value="false"),
            DeclareLaunchArgument("yolo_publish_image", default_value="true"),
            DeclareLaunchArgument("yolo_start_enabled", default_value="false"),
            DeclareLaunchArgument("yolo_camera_id", default_value="0"),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value=os.path.join(
                    nav2_config_dir, "config", "preset_waypoints.yaml"
                ),
            ),
            DeclareLaunchArgument("preset_mission_loop", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([nav2_config_dir, "launch", "mid360_navigation.py"])]
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "cmd_invert_linear_x": LaunchConfiguration("cmd_invert_linear_x"),
                    "cmd_invert_angular_z": LaunchConfiguration("cmd_invert_angular_z"),
                    "start_preset_mission": LaunchConfiguration("start_preset_mission"),
                    "start_yolo": LaunchConfiguration("start_yolo"),
                    "nav_start_rviz": LaunchConfiguration("nav_start_rviz"),
                    "yolo_show_detection": LaunchConfiguration("yolo_show_detection"),
                    "yolo_publish_image": LaunchConfiguration("yolo_publish_image"),
                    "yolo_start_enabled": LaunchConfiguration("yolo_start_enabled"),
                    "yolo_camera_id": LaunchConfiguration("yolo_camera_id"),
                    "waypoint_file": LaunchConfiguration("waypoint_file"),
                    "preset_mission_loop": LaunchConfiguration("preset_mission_loop"),
                }.items(),
            ),
        ]
    )
