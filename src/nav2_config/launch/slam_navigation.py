#!/usr/bin/python3
"""
SLAM + Navigation 同时运行

同时启动 Cartographer SLAM 和 Nav2 导航，实现：
1. 实时建图
2. 实时定位 (Cartographer)
3. 实时路径规划
4. 无需预先保存地图
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    """SLAM + Navigation 同时运行"""

    slam_config_dir = get_package_share_directory('slam_config')
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
        description='Start yesense IMU driver in this launch (set false when dog.launch is already running)'
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
        default_value='false',
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

    params_file = os.path.join(nav2_config_dir, 'config', 'nav2_slam_params.yaml')
    rviz_config = os.path.join(nav2_config_dir, 'rviz', 'slam_nav.rviz')
    nav_to_pose_bt_xml = os.path.join(
        nav2_config_dir,
        'behavior_trees',
        'navigate_to_pose_w_path_invalid_replanning_and_recovery.xml'
    )

    # ============================================================================
    # 1. 静态 TF
    # ============================================================================
    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([slam_config_dir, 'launch', 'tf_static_launch.py'])
        ]),
        launch_arguments={
            'imu_roll_deg': LaunchConfiguration('imu_roll_deg'),
            'imu_pitch_deg': LaunchConfiguration('imu_pitch_deg'),
            'imu_yaw_deg': LaunchConfiguration('imu_yaw_deg'),
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

    # ============================================================================
    # 2. 激光雷达驱动
    # ============================================================================
    lidar_driver = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lidar_uart_ros2', 'lsn10p.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    # ============================================================================
    # 3. IMU 驱动（ros_ws 原生 yesense）
    # ============================================================================

    # ============================================================================
    # 4. Cartographer SLAM
    # ============================================================================
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
            '--ros-args',
            '--log-level', 'warn',
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
            '--ros-args',
            '--log-level', 'warn',
        ]
    )

    # ============================================================================
    # 5. Controller Server (局部路径规划)
    # ============================================================================
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        arguments=['--ros-args', '--log-level', 'info']  # 保留 info 以显示导航状态
    )

    # ============================================================================
    # 6. Planner Server (全局路径规划)
    # ============================================================================
    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ============================================================================
    # 7. Behavior Server
    # ============================================================================
    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ============================================================================
    # 8. Smoother Server
    # ============================================================================
    smoother_server = LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    velocity_smoother = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ============================================================================
    # 9. BT Navigator (行为树控制)
    # ============================================================================
    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'default_nav_to_pose_bt_node_xml': nav_to_pose_bt_xml}
        ],
        remappings=[
            ('odom', '/odom'),  # 使用 Cartographer 发布的 odom
        ],
        arguments=['--ros-args', '--log-level', 'info']  # 保留导航状态信息
    )

    # ============================================================================
    # 10. Lifecycle Manager (自动管理 Nav2 节点生命周期)
    # ============================================================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': False},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'smoother_server',
                'velocity_smoother',
                'bt_navigator'
            ]}
        ]
    )

    nav2_activator = Node(
        package='nav2_config',
        executable='wait_for_odom_activate_nav2.py',
        name='wait_for_odom_activate_nav2',
        output='screen',
        parameters=[
            {'target_frame': 'odom'},
            {'source_frame': 'base_link'},
            {'lifecycle_service': '/lifecycle_manager_navigation/manage_nodes'},
        ]
    )

    cmd_vel_to_move_cmd = Node(
        package='nav2_config',
        executable='cmd_vel_to_move_cmd.py',
        name='cmd_vel_to_move_cmd',
        output='screen',
        parameters=[
            {'linear_x_scale': 1.0},
            {'angular_z_scale': 0.45},
            {'invert_linear_x': LaunchConfiguration('cmd_invert_linear_x')},
            {'invert_angular_z': LaunchConfiguration('cmd_invert_angular_z')},
            {'goal_pose_topic': '/goal_pose'},
            {'preset_goal_pose_topic': '/preset_current_goal'},
            {'plan_topic': '/plan'},
            {'global_plan_topic': '/global_plan'},
            {'base_frame': 'base_link'},
            {'final_align_enabled': True},
            {'final_align_xy_trigger': 0.08},
            {'final_align_yaw_trigger': 0.18},
            {'final_align_yaw_exit': 0.08},
            {'final_align_linear_scale': 0.15},
            {'final_align_max_linear_x': 0.02},
            {'final_align_angular_kp': 1.2},
            {'min_nonzero_angular_z': 0.06},                    # 比死区略大，防止末端wz太小不动
            {'min_nonzero_angular_linear_x_threshold': 0.02},   # 更早允许纯转向阶段触发最小角速度
            {'max_angular_z': 0.24},        # 与 velocity_smoother / RotationShim 对齐
            {'max_angular_z_accel': 0.35},  # 与 velocity_smoother 对齐，避免链路前后限幅打架
            {'smoothing_alpha': 0.50},      # 保留桥接层平滑，但减少指令迟滞
            {'max_step_x_rate': 2.4},       # 适度提高转向步态指令响应
            {'max_step_y_rate': 2.0},       # 适度提高直行步态指令响应
        ]
    )

    cmd_vel_visualizer = Node(
        package='nav2_config',
        executable='cmd_vel_visualizer.py',
        name='cmd_vel_visualizer',
        output='screen'
    )

    preset_waypoint_mission = Node(
        package='nav2_config',
        executable='preset_waypoint_mission.py',
        name='preset_waypoint_mission',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_preset_mission')),
        parameters=[
            {'waypoint_file': LaunchConfiguration('waypoint_file')},
            {'loop_mission': LaunchConfiguration('preset_mission_loop')},
            {'frame_id': 'map'},
        ]
    )

    # ============================================================================
    # 11. RViz2
    # ============================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
        tf_static,
        lidar_driver,
        yesense_launch,
        cartographer_node,
        occupancy_grid_node,
        controller_server,
        planner_server,
        behavior_server,
        smoother_server,
        velocity_smoother,
        bt_navigator,
        lifecycle_manager,
        nav2_activator,
        cmd_vel_to_move_cmd,
        cmd_vel_visualizer,
        preset_waypoint_mission,
        rviz_node,
    ])
