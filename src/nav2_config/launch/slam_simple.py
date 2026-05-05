#!/usr/bin/python3
"""
简化版 SLAM + 导航启动文件

使用 nav2_bringup 包简化配置
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """使用 nav2_bringup 启动 SLAM + 导航"""

    slam_config_dir = get_package_share_directory('slam_config')

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
        description='Static TF yaw from base_link_raw to gyro_link (degrees)'
    )

    laser_yaw_arg = DeclareLaunchArgument(
        'laser_yaw_deg',
        default_value='0.0',
        description='Static TF yaw from base_link_raw to laser (degrees)'
    )
    laser_x_arg = DeclareLaunchArgument(
        'laser_x',
        default_value='-0.12102',
        description='Static TF x from base_link to laser (meters)'
    )
    laser_y_arg = DeclareLaunchArgument(
        'laser_y',
        default_value='0.0',
        description='Static TF y from base_link to laser (meters)'
    )
    laser_z_arg = DeclareLaunchArgument(
        'laser_z',
        default_value='0.2',
        description='Static TF z from base_link to laser (meters)'
    )
    body_yaw_arg = DeclareLaunchArgument(
        'body_yaw_deg',
        default_value='90.0',
        description='Static TF yaw from base_link_raw to corrected base_link (degrees)'
    )

    # ============================================================================
    # 传感器驱动
    # ============================================================================
    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([slam_config_dir, 'launch', 'tf_static_launch.py'])
        ]),
        launch_arguments={
            'imu_roll_deg': LaunchConfiguration('imu_roll_deg'),
            'imu_pitch_deg': LaunchConfiguration('imu_pitch_deg'),
            'imu_yaw_deg': LaunchConfiguration('imu_yaw_deg'),
            'laser_yaw_deg': LaunchConfiguration('laser_yaw_deg'),
            'laser_x': LaunchConfiguration('laser_x'),
            'laser_y': LaunchConfiguration('laser_y'),
            'laser_z': LaunchConfiguration('laser_z'),
            'body_yaw_deg': LaunchConfiguration('body_yaw_deg'),
        }.items(),
    )

    yesense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('yesense_std_ros2'),
                'launch',
                'yesense_node.launch.py',
            ])
        ])
    )

    lidar_driver = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lidar_uart_ros2', 'lsn10p.yaml'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'self_filter_laser_x': LaunchConfiguration('laser_x'),
                'self_filter_laser_y': LaunchConfiguration('laser_y'),
                'self_filter_laser_yaw_deg': LaunchConfiguration('laser_yaw_deg'),
            }
        ],
    )

    # Cartographer 配置
    cartographer_config_dir = os.path.join(slam_config_dir, 'config')
    cartographer_config_basename = 'n10p_2d_simple.lua'

    if not os.path.exists(os.path.join(cartographer_config_dir, cartographer_config_basename)):
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

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # RViz
    rviz_config = os.path.join(get_package_share_directory('nav2_config'), 'rviz', 'slam_nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        imu_topic_arg,
        imu_roll_arg,
        imu_pitch_arg,
        imu_yaw_arg,
        laser_yaw_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        body_yaw_arg,
        tf_static,
        lidar_driver,
        yesense_launch,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])
