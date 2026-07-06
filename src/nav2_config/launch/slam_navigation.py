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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
        description='Static TF yaw from base_link_raw to gyro_link (degrees)'
    )

    laser_yaw_arg = DeclareLaunchArgument(
        'laser_yaw_deg',
        default_value='0.0',
        description='Static TF yaw from base_link_raw to laser (degrees)'
    )
    self_filter_laser_yaw_arg = DeclareLaunchArgument(
        'self_filter_laser_yaw_deg',
        default_value='180.0',
        description='LiDAR polar angle yaw used only by the lslidar self-filter (degrees)'
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

    start_preset_mission_arg = DeclareLaunchArgument(
        'start_preset_mission',
        default_value='false',
        description='Start preset multi-waypoint mission node'
    )

    start_field_reference_arg = DeclareLaunchArgument(
        'start_field_reference',
        default_value='true',
        description='Show the rule-based task-field reference overlay in RViz'
    )

    show_task_item_zones_arg = DeclareLaunchArgument(
        'show_task_item_zones',
        default_value='true',
        description='Show storage/place-zone blocks in the field reference overlay'
    )

    start_yolo_arg = DeclareLaunchArgument(
        'start_yolo',
        default_value='true',
        description='Start YOLO detection node together with navigation'
    )

    yolo_show_detection_arg = DeclareLaunchArgument(
        'yolo_show_detection',
        default_value='false',
        description='Show YOLO OpenCV window'
    )

    yolo_publish_image_arg = DeclareLaunchArgument(
        'yolo_publish_image',
        default_value='false',
        description='Publish YOLO image topic for RViz'
    )

    yolo_start_enabled_arg = DeclareLaunchArgument(
        'yolo_start_enabled',
        default_value='false',
        description='Whether YOLO inference starts immediately or waits for waypoint trigger'
    )

    yolo_camera_id_arg = DeclareLaunchArgument(
        'yolo_camera_id',
        default_value='0',
        description='Camera device index used by YOLO detection node'
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

    arm_start_settle_arg = DeclareLaunchArgument(
        'arm_start_settle_sec',
        default_value='1.0',
        description='Seconds to lock and stop the base before starting an arm waypoint task'
    )

    verify_reached_pose_arg = DeclareLaunchArgument(
        'verify_reached_pose',
        default_value='true',
        description='Verify map->base_link pose after Nav2 success before running waypoint side effects'
    )

    reached_xy_tolerance_arg = DeclareLaunchArgument(
        'reached_xy_tolerance',
        default_value='0.25',
        description='Maximum map-frame XY distance allowed after Nav2 reports waypoint success'
    )

    start_runtime_logger_arg = DeclareLaunchArgument(
        'start_runtime_logger',
        default_value='false',
        description='Record run_dog pose, command velocity, move_cmd, goal, and scan summary CSV logs'
    )

    runtime_log_dir_arg = DeclareLaunchArgument(
        'runtime_log_dir',
        default_value='/home/orangepi/run_dog_logs',
        description='Directory used by run_dog runtime logger'
    )

    runtime_log_rate_hz_arg = DeclareLaunchArgument(
        'runtime_log_rate_hz',
        default_value='10.0',
        description='CSV sampling rate used by run_dog runtime logger'
    )

    params_file = os.path.join(nav2_config_dir, 'config', 'nav2_slam_params.yaml')
    rviz_config = os.path.join(nav2_config_dir, 'rviz', 'slam_nav.rviz')
    nav_to_pose_bt_xml = os.path.join(
        nav2_config_dir,
        'behavior_trees',
        'navigate_to_pose_w_path_invalid_replanning_and_recovery.xml'
    )
    nav_through_poses_bt_xml = os.path.join(
        nav2_config_dir,
        'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml'
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

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('yolov11n_rknn'),
                'launch',
                'yolo_detection.launch.py',
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('start_yolo')),
        launch_arguments={
            'show_detection': LaunchConfiguration('yolo_show_detection'),
            'publish_image': LaunchConfiguration('yolo_publish_image'),
            'start_enabled': LaunchConfiguration('yolo_start_enabled'),
            'camera_id': LaunchConfiguration('yolo_camera_id'),
            'enable_topic': '/yolo/enable',
        }.items(),
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
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'self_filter_laser_x': LaunchConfiguration('laser_x'),
                'self_filter_laser_y': LaunchConfiguration('laser_y'),
                'self_filter_laser_yaw_deg': LaunchConfiguration('self_filter_laser_yaw_deg'),
            }
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
        arguments=['--ros-args', '--log-level', 'warn']
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
            {'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml},
            {'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml},
            {'default_nav_to_pose_bt_node_xml': nav_to_pose_bt_xml},
            {'default_nav_through_poses_bt_node_xml': nav_through_poses_bt_xml}
        ],
        remappings=[
            ('odom', '/odom'),  # 使用 Cartographer 发布的 odom
        ],
        arguments=['--ros-args', '--log-level', 'warn']
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
            {'status_topic': '/cmd_vel_to_move_cmd/status'},
            {'base_frame': 'base_link'},
            {'cmd_vel_timeout_sec': 0.20},  # 目标附近不要长时间保持上一条速度，避免越过目标还继续发
            {'goal_stop_guard_enabled': True},
            {'goal_stop_xy_tolerance': 0.10},
            {'final_align_enabled': False},  # 关闭 bridge 终点原地对齐，避免四足在目标点反复原地旋转
            {'min_nonzero_linear_x': 0.06},  # 低于 0.06m/s 四足实际不迈步，抬高非零前进命令
            {'final_align_xy_trigger': 0.08},
            {'final_align_yaw_trigger': 0.35},                  # 更早进入 final_align，给更多调整时间
            {'final_align_yaw_exit': 0.04},                     # 约 2.3° 才退出，提高最终朝向精度
            {'final_align_linear_scale': 0.15},
            {'final_align_max_linear_x': 0.0},                  # 完全原地旋转，不再前进
            {'final_align_angular_kp': 0.8},                    # 降低比例增益，减少高速旋转过冲
            {'min_nonzero_angular_z': 0.04},                    # 原地对准只做慢速微调
            {'min_nonzero_angular_linear_x_threshold': 0.02},   # 更早允许纯转向阶段触发最小角速度
            {'max_angular_z': 0.40},        # 与 velocity_smoother 对齐，限制最大角速度 0.4rad/s
            {'max_angular_z_accel': 0.20},  # 与 velocity_smoother 对齐，降低对准加速度
            {'smoothing_alpha': 0.30},      # 降低平滑，减少 final_align 阶段的指令迟滞
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

    runtime_logger = Node(
        package='nav2_config',
        executable='run_dog_runtime_logger.py',
        name='run_dog_runtime_logger',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_runtime_logger')),
        parameters=[
            {'log_root': LaunchConfiguration('runtime_log_dir')},
            {'sample_rate_hz': LaunchConfiguration('runtime_log_rate_hz')},
            {'map_frame': 'map'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'odom_topic': '/odom'},
            {'cmd_vel_nav_topic': '/cmd_vel_nav'},
            {'cmd_vel_topic': '/cmd_vel'},
            {'move_cmd_topic': '/move_cmd'},
            {'goal_topic': '/preset_current_goal'},
            {'scan_topic': '/scan'},
            {'bridge_status_topic': '/cmd_vel_to_move_cmd/status'},
        ],
    )

    preset_waypoint_mission = Node(
        package='nav2_config',
        executable='preset_waypoint_mission.py',
        name='preset_waypoint_mission',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('start_field_reference'), "' == 'true' or '",
            LaunchConfiguration('start_preset_mission'), "' == 'true'"
        ])),
        parameters=[
            {'waypoint_file': LaunchConfiguration('waypoint_file')},
            {'auto_start': LaunchConfiguration('start_preset_mission')},
            {'loop_mission': LaunchConfiguration('preset_mission_loop')},
            {'frame_id': 'map'},
            {'publish_field_layout': LaunchConfiguration('start_field_reference')},
            {'publish_task_item_zones': LaunchConfiguration('show_task_item_zones')},
            {'arm_start_settle_sec': LaunchConfiguration('arm_start_settle_sec')},
            {'verify_reached_pose': LaunchConfiguration('verify_reached_pose')},
            {'reached_xy_tolerance': LaunchConfiguration('reached_xy_tolerance')},
            {'same_position_spin_enabled': True},
            {'same_position_tolerance': 0.05},
            {'same_yaw_tolerance_deg': 5.0},
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
        laser_yaw_arg,
        self_filter_laser_yaw_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        body_yaw_arg,
        start_preset_mission_arg,
        start_field_reference_arg,
        show_task_item_zones_arg,
        start_yolo_arg,
        yolo_show_detection_arg,
        yolo_publish_image_arg,
        yolo_start_enabled_arg,
        yolo_camera_id_arg,
        waypoint_file_arg,
        preset_mission_loop_arg,
        arm_start_settle_arg,
        verify_reached_pose_arg,
        reached_xy_tolerance_arg,
        start_runtime_logger_arg,
        runtime_log_dir_arg,
        runtime_log_rate_hz_arg,
        tf_static,
        lidar_driver,
        yesense_launch,
        yolo_launch,
        TimerAction(
            period=1.5,
            actions=[
                cartographer_node,
                occupancy_grid_node,
            ],
        ),
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
        runtime_logger,
        preset_waypoint_mission,
        rviz_node,
    ])
