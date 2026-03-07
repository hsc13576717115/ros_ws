#!/usr/bin/python3
"""
Cartographer 2D SLAM launch file for N10P LiDAR with IMU fusion

This launch file starts:
1. Static TF publishers
2. N10P LiDAR driver
3. IMU driver
4. Cartographer SLAM node
5. Occupancy grid node
6. RViz2 visualization

Usage:
    ros2 launch slam_config slam_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Generate launch description for Cartographer 2D SLAM"""

    # Get package directories
    slam_config_dir = get_package_share_directory('slam_config')
    lslidar_dir = get_package_share_directory('lslidar_driver')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data_raw',
        description='IMU topic used by Cartographer'
    )

    start_imu_arg = DeclareLaunchArgument(
        'start_imu',
        default_value='false',
        description='Start yesense IMU driver in this launch'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of the occupancy grid (meters)'
    )

    publish_period_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Period of occupancy grid publishing (seconds)'
    )

    # ============================================================================
    # 1. Static TF publishers
    # ============================================================================
    tf_static_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([slam_config_dir, 'launch', 'tf_static_launch.py'])
        ])
    )

    yesense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('yesense_std_ros2'),
                'launch',
                'yesense_node.launch.py',
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('start_imu'))
    )

    # ============================================================================
    # 2. N10P LiDAR driver (serial version)
    # ============================================================================
    # Change to lsn10p_net_launch.py for network version
    lidar_config_file = os.path.join(
        lslidar_dir, 'params', 'lidar_uart_ros2', 'lsn10p.yaml'
    )

    lidar_driver = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            lidar_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    # ============================================================================
    # 3. IMU driver (ros_ws original yesense)
    # ============================================================================

    # ============================================================================
    # 4. Cartographer SLAM node
    # ============================================================================
    # Try to use custom config first, fallback to cartographer default
    cartographer_config_dir = os.path.join(slam_config_dir, 'config')
    cartographer_config_basename = 'n10p_2d_simple.lua'

    if not os.path.exists(os.path.join(cartographer_config_dir, cartographer_config_basename)):
        # Use revo_lds which is designed for single laser scan
        cartographer_config_dir = os.path.join(
            get_package_share_directory('cartographer_ros'),
            'configuration_files'
        )
        cartographer_config_basename = 'revo_lds.lua'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('imu', LaunchConfiguration('imu_topic')),
        ],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename,
        ]
    )

    # ============================================================================
    # 5. Occupancy grid node
    # ============================================================================
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-resolution', LaunchConfiguration('resolution'),
            '-publish_period_sec', LaunchConfiguration('publish_period_sec')
        ]
    )

    # ============================================================================
    # 6. RViz2 visualization
    # ============================================================================
    rviz_config_file = os.path.join(slam_config_dir, 'rviz', 'slam_2d.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ============================================================================
    # Launch description
    # ============================================================================
    return LaunchDescription([
        use_sim_time_arg,
        imu_topic_arg,
        start_imu_arg,
        resolution_arg,
        publish_period_arg,

        # Launch nodes in sequence
        tf_static_launch,
        lidar_driver,
        yesense_launch,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])
