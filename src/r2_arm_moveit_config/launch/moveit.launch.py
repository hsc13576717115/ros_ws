import os
import shutil

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    control_share = get_package_share_directory("r2_arm_control")
    moveit_share = get_package_share_directory("r2_arm_moveit_config")

    motors_yaml = LaunchConfiguration("motors_yaml")
    arm_yaml = LaunchConfiguration("arm_yaml")
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    start_rviz = LaunchConfiguration("start_rviz").perform(context).strip().lower()
    use_sim_time = LaunchConfiguration("use_sim_time")
    arm_config = _load_yaml(LaunchConfiguration("arm_yaml").perform(context))
    executor_mode = str(arm_config.get("executor_mode", "direct_joint_trajectory")).strip()
    raw_joint_states_topic = arm_config.get("joint_states_topic", "/joint_states")
    moveit_joint_states_topic = arm_config.get("moveit_joint_states_topic", "/joint_states_moveit")
    start_rviz_enabled = start_rviz in ("1", "true", "yes", "on")
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

    urdf_path = os.path.join(control_share, "urdf", "r2_arm.urdf.xacro")
    srdf_path = os.path.join(moveit_share, "srdf", "r2_arm.srdf")
    kinematics_path = os.path.join(moveit_share, "config", "kinematics.yaml")
    ompl_path = os.path.join(moveit_share, "config", "ompl_planning.yaml")
    joint_limits_path = os.path.join(moveit_share, "config", "joint_limits.yaml")
    controllers_path = os.path.join(moveit_share, "config", "moveit_controllers.yaml")

    xacro_executable = shutil.which("xacro") or "xacro"
    robot_description_content = Command(
        [
            xacro_executable,
            " ",
            urdf_path,
            " motors_yaml:=",
            motors_yaml,
            " arm_yaml:=",
            arm_yaml,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    with open(srdf_path, "r", encoding="utf-8") as handle:
        robot_description_semantic = {"robot_description_semantic": handle.read()}

    kinematics_yaml = _load_yaml(kinematics_path)
    ompl_yaml = _load_yaml(ompl_path)
    joint_limits_yaml = _load_yaml(joint_limits_path)
    controllers_yaml = _load_yaml(controllers_path)

    planning_scene_monitor_parameters = {
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options.joint_state_topic": moveit_joint_states_topic,
        "planning_scene_monitor_options.wait_for_initial_state_timeout": 15.0,
    }

    trajectory_execution_parameters = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": arm_config.get(
            "moveit_allowed_execution_duration_scaling", 3.0
        ),
        "trajectory_execution.allowed_goal_duration_margin": arm_config.get(
            "moveit_allowed_goal_duration_margin", 2.0
        ),
        "trajectory_execution.allowed_start_tolerance": arm_config.get(
            "moveit_allowed_start_tolerance", 0.10
        ),
        "use_sim_time": use_sim_time,
    }

    control_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control_share, "launch", "bringup.launch.py")),
        launch_arguments={
            "motors_yaml": motors_yaml,
            "arm_yaml": arm_yaml,
            "start_ros2_control": "true",
            "start_command_server": "false",
            "start_pose_test_node": "false",
            "start_rviz": "false",
        }.items(),
    )

    joint_state_bridge_node = Node(
        package="r2_arm_control",
        executable="joint_state_timestamp_bridge_node",
        parameters=[
            {
                "input_topic": raw_joint_states_topic,
                "output_topic": moveit_joint_states_topic,
                "always_restamp": True,
            },
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=[("/joint_states", moveit_joint_states_topic)],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_yaml,
            controllers_yaml,
            planning_scene_monitor_parameters,
            trajectory_execution_parameters,
        ],
    )

    move_to_xz_service_node = Node(
        package="r2_arm_control",
        executable="arm_command_server_node",
        parameters=[
            arm_config,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    end_effector_state_node = Node(
        package="r2_arm_control",
        executable="ee_state_publisher_node",
        parameters=[
            arm_config,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    delayed_move_group_node = TimerAction(period=8.0, actions=[move_group_node])
    delayed_move_to_xz_service_node = TimerAction(period=12.0, actions=[move_to_xz_service_node])

    actions = [
        control_bringup,
        joint_state_bridge_node,
        end_effector_state_node,
        delayed_move_to_xz_service_node,
    ]

    if executor_mode == "mit_native_5var":
        actions.append(
            LogInfo(
                msg=(
                    "executor_mode=mit_native_5var: skipping move_group startup because "
                    "the move_to_xz service now drives the MIT command stream directly."
                )
            )
        )
    else:
        actions.append(delayed_move_group_node)

    if start_rviz_enabled:
      if has_display:
        actions.append(rviz_node)
      else:
        actions.append(
            LogInfo(
                msg=(
                    "start_rviz is true, but no DISPLAY/WAYLAND_DISPLAY is set. "
                    "Skipping RViz startup."
                )
            )
        )

    return actions


def generate_launch_description():
    control_share = get_package_share_directory("r2_arm_control")
    moveit_share = get_package_share_directory("r2_arm_moveit_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motors_yaml",
                default_value=os.path.join(control_share, "config", "motors.yaml"),
            ),
            DeclareLaunchArgument(
                "arm_yaml",
                default_value=os.path.join(control_share, "config", "arm.yaml"),
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(moveit_share, "rviz", "moveit.rviz"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
