#!/usr/bin/python3
"""
Static transform publisher for SLAM configuration
Publishes fixed TF between base_link and sensors
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for static TF"""

    config_file = os.path.join(
        get_package_share_directory('slam_config'),
        'config',
        'tf_static.yaml'
    )

    imu_roll_arg = DeclareLaunchArgument(
        'imu_roll_deg',
        default_value='0.0',
        description='Static TF roll from base_link_raw to gyro_link (degrees)',
    )
    imu_pitch_arg = DeclareLaunchArgument(
        'imu_pitch_deg',
        default_value='0.0',
        description='Static TF pitch from base_link_raw to gyro_link (degrees)',
    )
    imu_yaw_arg = DeclareLaunchArgument(
        'imu_yaw_deg',
        default_value='0.0',
        description='Static TF yaw from base_link_raw to gyro_link (degrees)',
    )
    laser_roll_arg = DeclareLaunchArgument(
        'laser_roll_deg',
        default_value='0.0',
        description='Static TF roll from base_link_raw to laser (degrees)',
    )
    laser_pitch_arg = DeclareLaunchArgument(
        'laser_pitch_deg',
        default_value='0.0',
        description='Static TF pitch from base_link_raw to laser (degrees)',
    )
    laser_yaw_arg = DeclareLaunchArgument(
        'laser_yaw_deg',
        default_value='0.0',
        description='Static TF yaw from base_link_raw to laser (degrees)',
    )
    laser_x_arg = DeclareLaunchArgument(
        'laser_x',
        default_value='-0.12102',
        description='Static TF x from base_link_raw to laser (meters)',
    )
    laser_y_arg = DeclareLaunchArgument(
        'laser_y',
        default_value='0.0',
        description='Static TF y from base_link_raw to laser (meters)',
    )
    laser_z_arg = DeclareLaunchArgument(
        'laser_z',
        default_value='0.2',
        description='Static TF z from base_link_raw to laser (meters)',
    )

    def degrees_to_radians(value):
        return PythonExpression([
            'float("',
            value,
            '") * 3.141592653589793 / 180.0',
        ])

    body_yaw_arg = DeclareLaunchArgument(
        'body_yaw_deg',
        default_value='90.0',
        description='Static TF yaw from base_link_raw to ROS-standard base_link (degrees)',
    )

    # base_link_raw -> base_link transform
    tf_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_body_axes',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', degrees_to_radians(LaunchConfiguration('body_yaw_deg')),
            '--frame-id', 'base_link_raw', '--child-frame-id', 'base_link',
        ]
    )

    # base_link_raw -> laser transform
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser',
        parameters=[config_file, {'use_yaml_as_config': True}],
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--roll', degrees_to_radians(LaunchConfiguration('laser_roll_deg')),
            '--pitch', degrees_to_radians(LaunchConfiguration('laser_pitch_deg')),
            '--yaw', degrees_to_radians(LaunchConfiguration('laser_yaw_deg')),
            '--frame-id', 'base_link_raw', '--child-frame-id', 'laser',
        ]
    )

    # base_link_raw -> gyro_link transform
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu',
        parameters=[config_file, {'use_yaml_as_config': True}],
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.1',
            '--roll', degrees_to_radians(LaunchConfiguration('imu_roll_deg')),
            '--pitch', degrees_to_radians(LaunchConfiguration('imu_pitch_deg')),
            '--yaw', degrees_to_radians(LaunchConfiguration('imu_yaw_deg')),
            '--frame-id', 'base_link_raw', '--child-frame-id', 'gyro_link',
        ]
    )

    return LaunchDescription([
        imu_roll_arg,
        imu_pitch_arg,
        imu_yaw_arg,
        laser_roll_arg,
        laser_pitch_arg,
        laser_yaw_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        body_yaw_arg,
        tf_body,
        tf_laser,
        tf_imu,
    ])
