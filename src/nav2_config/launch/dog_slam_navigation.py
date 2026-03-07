#!/usr/bin/python3

"""
Navigation launch for running together with dog.launch.py.

Expected usage:
1. Terminal A: ros2 launch vmc_quadruped_controller dog.launch.py
2. Terminal B: ros2 launch nav2_config dog_slam_navigation.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    nav2_config_dir = get_package_share_directory('nav2_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data_raw',
        description='IMU topic used by Cartographer'
    )

    start_imu_arg = DeclareLaunchArgument(
        'start_imu',
        default_value='false',
        description='Start yesense IMU driver in this launch (keep false when dog.launch.py is running)'
    )

    cmd_invert_linear_x_arg = DeclareLaunchArgument(
        'cmd_invert_linear_x',
        default_value='true',
        description='Invert /cmd_vel linear.x before mapping to /move_cmd step_y'
    )

    cmd_invert_angular_z_arg = DeclareLaunchArgument(
        'cmd_invert_angular_z',
        default_value='false',
        description='Invert /cmd_vel angular.z before mapping to /move_cmd step_x'
    )

    imu_roll_arg = DeclareLaunchArgument(
        'imu_roll_deg',
        default_value='0.0',
        description='Static TF roll from base_link to gyro_link (degrees)'
    )

    imu_pitch_arg = DeclareLaunchArgument(
        'imu_pitch_deg',
        default_value='0.0',
        description='Static TF pitch from base_link to gyro_link (degrees)'
    )

    imu_yaw_arg = DeclareLaunchArgument(
        'imu_yaw_deg',
        default_value='0.0',
        description='Static TF yaw from base_link to gyro_link (degrees)'
    )

    start_preset_mission_arg = DeclareLaunchArgument(
        'start_preset_mission',
        default_value='true',
        description='Start preset multi-waypoint mission node'
    )

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=os.path.join(nav2_config_dir, 'config', 'preset_waypoints.yaml'),
        description='YAML file path for preset waypoints'
    )

    preset_mission_loop_arg = DeclareLaunchArgument(
        'preset_mission_loop',
        default_value='false',
        description='Loop preset mission after the last waypoint'
    )

    return LaunchDescription([
        use_sim_time_arg,
        imu_topic_arg,
        start_imu_arg,
        cmd_invert_linear_x_arg,
        cmd_invert_angular_z_arg,
        imu_roll_arg,
        imu_pitch_arg,
        imu_yaw_arg,
        start_preset_mission_arg,
        waypoint_file_arg,
        preset_mission_loop_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_config_dir, 'launch', 'slam_navigation.py'])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'start_imu': LaunchConfiguration('start_imu'),
                'cmd_invert_linear_x': LaunchConfiguration('cmd_invert_linear_x'),
                'cmd_invert_angular_z': LaunchConfiguration('cmd_invert_angular_z'),
                'imu_roll_deg': LaunchConfiguration('imu_roll_deg'),
                'imu_pitch_deg': LaunchConfiguration('imu_pitch_deg'),
                'imu_yaw_deg': LaunchConfiguration('imu_yaw_deg'),
                'start_preset_mission': LaunchConfiguration('start_preset_mission'),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'preset_mission_loop': LaunchConfiguration('preset_mission_loop'),
            }.items(),
        ),
    ])
