#!/usr/bin/python3

"""
One-shot launch for the quadruped body, SLAM, Nav2, and preset mission.

If the quadruped body stack is already running, pass start_dog:=false.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    nav2_config_dir = get_package_share_directory('nav2_config')
    vmc_config_dir = get_package_share_directory('vmc_quadruped_controller')

    start_dog_arg = DeclareLaunchArgument(
        'start_dog',
        default_value='true',
        description='Start vmc_quadruped_controller dog.launch.py before navigation'
    )

    start_arm_arg = DeclareLaunchArgument(
        'start_arm',
        default_value='true',
        description='Start arm MoveIt/control stack from dog.launch.py'
    )

    start_dpad_gpio_arg = DeclareLaunchArgument(
        'start_dpad_gpio',
        default_value='true',
        description='Start arm state machine / GPIO stack from dog.launch.py'
    )

    start_joy_arg = DeclareLaunchArgument(
        'start_joy',
        default_value='true',
        description='Start joystick node from dog.launch.py'
    )

    start_body_arg = DeclareLaunchArgument(
        'start_body',
        default_value='true',
        description='Start quadruped body controller from dog.launch.py'
    )

    start_dog_imu_arg = DeclareLaunchArgument(
        'start_dog_imu',
        default_value='true',
        description='Start yesense IMU driver from dog.launch.py'
    )

    imu_pitch_sign_arg = DeclareLaunchArgument(
        'imu_pitch_sign',
        default_value='-1.0',
        description='IMU pitch sign passed to the quadruped body controller'
    )

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
        description='Start yesense IMU driver inside slam_navigation.py; keep false when start_dog:=true'
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
        default_value='true',
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
        default_value='true',
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

    dog_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([vmc_config_dir, 'launch', 'dog.launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration('start_dog')),
        launch_arguments={
            'start_arm': LaunchConfiguration('start_arm'),
            'start_dpad_gpio': LaunchConfiguration('start_dpad_gpio'),
            'start_joy': LaunchConfiguration('start_joy'),
            'start_body': LaunchConfiguration('start_body'),
            'start_imu': LaunchConfiguration('start_dog_imu'),
            'imu_pitch_sign': LaunchConfiguration('imu_pitch_sign'),
        }.items(),
    )

    def slam_launch_include():
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_config_dir, 'launch', 'slam_navigation.py'])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                # dog.launch.py already owns the yesense IMU when start_dog is true.
                # Keep the SLAM include from starting a second yesense node on the
                # same serial port/topic; duplicate IMU publishers can crash Cartographer.
                'start_imu': 'false',
                'cmd_invert_linear_x': LaunchConfiguration('cmd_invert_linear_x'),
                'cmd_invert_angular_z': LaunchConfiguration('cmd_invert_angular_z'),
                'imu_roll_deg': LaunchConfiguration('imu_roll_deg'),
                'imu_pitch_deg': LaunchConfiguration('imu_pitch_deg'),
                'imu_yaw_deg': LaunchConfiguration('imu_yaw_deg'),
                'laser_yaw_deg': LaunchConfiguration('laser_yaw_deg'),
                'self_filter_laser_yaw_deg': LaunchConfiguration('self_filter_laser_yaw_deg'),
                'laser_x': LaunchConfiguration('laser_x'),
                'laser_y': LaunchConfiguration('laser_y'),
                'laser_z': LaunchConfiguration('laser_z'),
                'body_yaw_deg': LaunchConfiguration('body_yaw_deg'),
                'start_preset_mission': LaunchConfiguration('start_preset_mission'),
                'start_field_reference': LaunchConfiguration('start_field_reference'),
                'show_task_item_zones': LaunchConfiguration('show_task_item_zones'),
                'start_yolo': LaunchConfiguration('start_yolo'),
                'yolo_show_detection': LaunchConfiguration('yolo_show_detection'),
                'yolo_publish_image': LaunchConfiguration('yolo_publish_image'),
                'yolo_start_enabled': LaunchConfiguration('yolo_start_enabled'),
                'yolo_camera_id': LaunchConfiguration('yolo_camera_id'),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'preset_mission_loop': LaunchConfiguration('preset_mission_loop'),
            }.items(),
        )

    return LaunchDescription([
        start_dog_arg,
        start_arm_arg,
        start_dpad_gpio_arg,
        start_joy_arg,
        start_body_arg,
        start_dog_imu_arg,
        imu_pitch_sign_arg,
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
        dog_launch,
        TimerAction(
            period=8.0,
            actions=[slam_launch_include()],
            condition=IfCondition(LaunchConfiguration('start_dog')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_config_dir, 'launch', 'slam_navigation.py'])
            ]),
            condition=UnlessCondition(LaunchConfiguration('start_dog')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'start_imu': LaunchConfiguration('start_imu'),
                'cmd_invert_linear_x': LaunchConfiguration('cmd_invert_linear_x'),
                'cmd_invert_angular_z': LaunchConfiguration('cmd_invert_angular_z'),
                'imu_roll_deg': LaunchConfiguration('imu_roll_deg'),
                'imu_pitch_deg': LaunchConfiguration('imu_pitch_deg'),
                'imu_yaw_deg': LaunchConfiguration('imu_yaw_deg'),
                'laser_yaw_deg': LaunchConfiguration('laser_yaw_deg'),
                'self_filter_laser_yaw_deg': LaunchConfiguration('self_filter_laser_yaw_deg'),
                'laser_x': LaunchConfiguration('laser_x'),
                'laser_y': LaunchConfiguration('laser_y'),
                'laser_z': LaunchConfiguration('laser_z'),
                'body_yaw_deg': LaunchConfiguration('body_yaw_deg'),
                'start_preset_mission': LaunchConfiguration('start_preset_mission'),
                'start_field_reference': LaunchConfiguration('start_field_reference'),
                'show_task_item_zones': LaunchConfiguration('show_task_item_zones'),
                'start_yolo': LaunchConfiguration('start_yolo'),
                'yolo_show_detection': LaunchConfiguration('yolo_show_detection'),
                'yolo_publish_image': LaunchConfiguration('yolo_publish_image'),
                'yolo_start_enabled': LaunchConfiguration('yolo_start_enabled'),
                'yolo_camera_id': LaunchConfiguration('yolo_camera_id'),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'preset_mission_loop': LaunchConfiguration('preset_mission_loop'),
            }.items(),
        ),
    ])
