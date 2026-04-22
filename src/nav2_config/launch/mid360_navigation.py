#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    nav2_config_dir = get_package_share_directory("nav2_config")

    params_file = os.path.join(nav2_config_dir, "config", "nav2_fastlio_params.yaml")
    pointcloud_scan_config = os.path.join(
        nav2_config_dir, "config", "pointcloud_to_scan.yaml"
    )
    rviz_config = os.path.join(nav2_config_dir, "rviz", "slam_nav.rviz")
    nav_to_pose_bt_xml = os.path.join(
        nav2_config_dir,
        "behavior_trees",
        "navigate_to_pose_w_path_invalid_replanning_and_recovery.xml",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )
    cmd_invert_linear_x_arg = DeclareLaunchArgument(
        "cmd_invert_linear_x",
        default_value="true",
        description="Invert /cmd_vel linear.x before mapping to /move_cmd step_y",
    )
    cmd_invert_angular_z_arg = DeclareLaunchArgument(
        "cmd_invert_angular_z",
        default_value="false",
        description="Invert /cmd_vel angular.z before mapping to /move_cmd step_x",
    )
    start_preset_mission_arg = DeclareLaunchArgument(
        "start_preset_mission",
        default_value="false",
        description="Start preset multi-waypoint mission node",
    )
    start_yolo_arg = DeclareLaunchArgument(
        "start_yolo",
        default_value="true",
        description="Start YOLO detection node together with navigation",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "nav_start_rviz",
        default_value="false",
        description="Start RViz with the navigation view",
    )
    yolo_show_detection_arg = DeclareLaunchArgument(
        "yolo_show_detection", default_value="false"
    )
    yolo_publish_image_arg = DeclareLaunchArgument(
        "yolo_publish_image", default_value="true"
    )
    yolo_start_enabled_arg = DeclareLaunchArgument(
        "yolo_start_enabled", default_value="false"
    )
    yolo_camera_id_arg = DeclareLaunchArgument("yolo_camera_id", default_value="0")
    waypoint_file_arg = DeclareLaunchArgument(
        "waypoint_file",
        default_value=os.path.join(nav2_config_dir, "config", "preset_waypoints.yaml"),
    )
    preset_mission_loop_arg = DeclareLaunchArgument(
        "preset_mission_loop", default_value="false"
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file,
        description="Nav2 parameters for Mid360 + FAST-LIO mode",
    )
    fastlio_config_name_arg = DeclareLaunchArgument(
        "fastlio_config_name",
        default_value="fastlio_mid360_nav.yaml",
        description="FAST-LIO config file stored in nav2_config/config",
    )
    pointcloud_scan_config_arg = DeclareLaunchArgument(
        "pointcloud_scan_config",
        default_value=pointcloud_scan_config,
        description="Pointcloud-to-scan projection parameters",
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("livox_ros_driver2"),
                        "launch_ROS2",
                        "msg_MID360_launch.py",
                    ]
                )
            ]
        )
    )

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [get_package_share_directory("fast_lio"), "launch", "mapping.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "config_path": os.path.join(nav2_config_dir, "config"),
            "config_file": LaunchConfiguration("fastlio_config_name"),
            "rviz": "false",
        }.items(),
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--qx",
            "0.0",
            "--qy",
            "0.0",
            "--qz",
            "0.0",
            "--qw",
            "1.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    base_to_livox_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_livox_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.2",
            "--qx",
            "0.0",
            "--qy",
            "0.0",
            "--qz",
            "0.0",
            "--qw",
            "1.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "livox_frame",
        ],
    )

    pointcloud_to_scan = Node(
        package="nav2_config",
        executable="pointcloud_to_scan.py",
        name="pointcloud_to_scan",
        output="screen",
        parameters=[
            LaunchConfiguration("pointcloud_scan_config"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("yolov11n_rknn"),
                        "launch",
                        "yolo_detection.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(LaunchConfiguration("start_yolo")),
        launch_arguments={
            "show_detection": LaunchConfiguration("yolo_show_detection"),
            "publish_image": LaunchConfiguration("yolo_publish_image"),
            "start_enabled": LaunchConfiguration("yolo_start_enabled"),
            "camera_id": LaunchConfiguration("yolo_camera_id"),
            "enable_topic": "/yolo/enable",
        }.items(),
    )

    controller_server = LifecycleNode(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[("cmd_vel", "cmd_vel_nav")],
        arguments=["--ros-args", "--log-level", "info"],
    )

    planner_server = LifecycleNode(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    behavior_server = LifecycleNode(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    smoother_server = LifecycleNode(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    velocity_smoother = LifecycleNode(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    bt_navigator = LifecycleNode(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"default_nav_to_pose_bt_node_xml": nav_to_pose_bt_xml},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"autostart": False},
            {
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "smoother_server",
                    "velocity_smoother",
                    "bt_navigator",
                ]
            },
        ],
    )

    nav2_activator = Node(
        package="nav2_config",
        executable="wait_for_odom_activate_nav2.py",
        name="wait_for_odom_activate_nav2",
        output="screen",
        parameters=[
            {"target_frame": "odom"},
            {"source_frame": "base_link"},
            {"lifecycle_service": "/lifecycle_manager_navigation/manage_nodes"},
        ],
    )

    cmd_vel_to_move_cmd = Node(
        package="nav2_config",
        executable="cmd_vel_to_move_cmd.py",
        name="cmd_vel_to_move_cmd",
        output="screen",
        parameters=[
            {"linear_x_scale": 1.0},
            {"angular_z_scale": 0.45},
            {"invert_linear_x": LaunchConfiguration("cmd_invert_linear_x")},
            {"invert_angular_z": LaunchConfiguration("cmd_invert_angular_z")},
            {"goal_pose_topic": "/goal_pose"},
            {"preset_goal_pose_topic": "/preset_current_goal"},
            {"plan_topic": "/plan"},
            {"global_plan_topic": "/global_plan"},
            {"base_frame": "base_link"},
            {"final_align_enabled": True},
            {"final_align_xy_trigger": 0.08},
            {"final_align_yaw_trigger": 0.18},
            {"final_align_yaw_exit": 0.08},
            {"final_align_linear_scale": 0.15},
            {"final_align_max_linear_x": 0.02},
            {"final_align_angular_kp": 1.2},
            {"min_nonzero_angular_z": 0.06},
            {"min_nonzero_angular_linear_x_threshold": 0.02},
            {"max_angular_z": 0.24},
            {"max_angular_z_accel": 0.35},
            {"smoothing_alpha": 0.50},
            {"max_step_x_rate": 2.4},
            {"max_step_y_rate": 2.0},
        ],
    )

    cmd_vel_visualizer = Node(
        package="nav2_config",
        executable="cmd_vel_visualizer.py",
        name="cmd_vel_visualizer",
        output="screen",
    )

    preset_waypoint_mission = Node(
        package="nav2_config",
        executable="preset_waypoint_mission.py",
        name="preset_waypoint_mission",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_preset_mission")),
        parameters=[
            {"waypoint_file": LaunchConfiguration("waypoint_file")},
            {"loop_mission": LaunchConfiguration("preset_mission_loop")},
            {"frame_id": "map"},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="nav_rviz2",
        output="screen",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", "warn"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("nav_start_rviz")),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            cmd_invert_linear_x_arg,
            cmd_invert_angular_z_arg,
            start_preset_mission_arg,
            start_yolo_arg,
            start_rviz_arg,
            yolo_show_detection_arg,
            yolo_publish_image_arg,
            yolo_start_enabled_arg,
            yolo_camera_id_arg,
            waypoint_file_arg,
            preset_mission_loop_arg,
            params_file_arg,
            fastlio_config_name_arg,
            pointcloud_scan_config_arg,
            map_to_odom_tf,
            base_to_livox_tf,
            livox_launch,
            fastlio_launch,
            pointcloud_to_scan,
            yolo_launch,
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
        ]
    )
