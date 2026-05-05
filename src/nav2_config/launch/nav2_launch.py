#!/usr/bin/python3
"""
Nav2 Navigation Launch File

Brings up the navigation stack for autonomous navigation using:
- Map server
- AMCL localization
- Planner (A* / NavFn)
- Controller (DWB / Regulated Pure Pursuit)
- Costmaps (global + local)
- Behavior tree
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Nav2 navigation"""

    nav2_config_dir = get_package_share_directory('nav2_config')
    slam_config_dir = get_package_share_directory('slam_config')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )

    start_imu_arg = DeclareLaunchArgument(
        'start_imu',
        default_value='false',
        description='Start yesense IMU driver in this launch (set false when dog.launch is already running)'
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

    cmd_invert_linear_x_arg = DeclareLaunchArgument(
        'cmd_invert_linear_x',
        default_value='true',
        description='Invert /cmd_vel linear.x before mapping to /move_cmd step_y'
    )

    cmd_invert_angular_z_arg = DeclareLaunchArgument(
        'cmd_invert_angular_z',
        default_value='true',
        description='Invert /cmd_vel angular.z before mapping to /move_cmd step_x'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the nav2 params file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(nav2_config_dir, 'rviz', 'nav2_view.rviz'),
        description='Full path to the RViz config file'
    )

    # ============================================================================
    # Include SLAM sensors (TF, LiDAR, IMU)
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
        ]),
        condition=IfCondition(LaunchConfiguration('start_imu'))
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

    # ============================================================================
    # Map Server
    # ============================================================================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'yaml_filename': LaunchConfiguration('map')}
        ]
    )

    # ============================================================================
    # AMCL Localization
    # ============================================================================
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # ============================================================================
    # Lifecycle Manager
    # ============================================================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'smoother_server',
                    'velocity_smoother',
                    'bt_navigator',
                ]
            }
        ]
    )

    # ============================================================================
    # Controller Server
    # ============================================================================
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # ============================================================================
    # Planner Server
    # ============================================================================
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # ============================================================================
    # Behavior Server
    # ============================================================================
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # ============================================================================
    # Smoother Server
    # ============================================================================
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    # ============================================================================
    # BT Navigator
    # ============================================================================
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    cmd_vel_to_move_cmd = Node(
        package='nav2_config',
        executable='cmd_vel_to_move_cmd.py',
        name='cmd_vel_to_move_cmd',
        output='screen',
        parameters=[
            {'invert_linear_x': LaunchConfiguration('cmd_invert_linear_x')},
            {'invert_angular_z': LaunchConfiguration('cmd_invert_angular_z')},
            {'min_nonzero_angular_z': 0.06},                    # 比死区略大，防止末端wz太小不动
            {'min_nonzero_angular_linear_x_threshold': 0.03},   # 仅原地对齐阶段启用最小角速度
            {'max_angular_z': 0.24},        # 小幅放宽导航角速度限幅
            {'max_angular_z_accel': 0.35},  # 小幅放宽导航角加速度限幅
        ]
    )

    cmd_vel_visualizer = Node(
        package='nav2_config',
        executable='cmd_vel_visualizer.py',
        name='cmd_vel_visualizer',
        output='screen'
    )

    # ============================================================================
    # RViz2 with Nav2 plugins
    # ============================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ============================================================================
    # Launch Description
    # ============================================================================
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        autostart_arg,
        map_arg,
        start_imu_arg,
        imu_roll_arg,
        imu_pitch_arg,
        imu_yaw_arg,
        laser_yaw_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        body_yaw_arg,
        cmd_invert_linear_x_arg,
        cmd_invert_angular_z_arg,
        params_file_arg,
        rviz_config_arg,

        # Sensors
        tf_static,
        lidar_driver,
        yesense_launch,

        # Nav2 Nodes
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        smoother_server,
        velocity_smoother,
        bt_navigator,
        lifecycle_manager,
        cmd_vel_to_move_cmd,
        cmd_vel_visualizer,
        rviz_node,
    ])
