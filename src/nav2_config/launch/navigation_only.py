#!/usr/bin/python3
"""
Navigation Launch - Use existing map

This launch file assumes you already have a map and want to run
localization + navigation only.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Navigation launch for existing map"""

    nav2_config_dir = get_package_share_directory('nav2_config')

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file (required for localization)'
    )

    start_imu_arg = DeclareLaunchArgument(
        'start_imu',
        default_value='false',
        description='Start yesense IMU driver in navigation launch'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml'),
        description='Path to nav2 params file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(nav2_config_dir, 'rviz', 'nav2_view.rviz'),
        description='Path to RViz config file'
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        start_imu_arg,
        params_file_arg,
        rviz_config_arg,

        # Include full navigation launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_config_dir, 'launch', 'nav2_launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map': LaunchConfiguration('map'),
                'start_imu': LaunchConfiguration('start_imu'),
                'params_file': LaunchConfiguration('params_file'),
                'rviz_config': LaunchConfiguration('rviz_config'),
                'autostart': 'true',
            }.items()
        ),
    ])
