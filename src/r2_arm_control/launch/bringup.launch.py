import os
import shutil
import textwrap

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from xml.sax.saxutils import escape


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _build_robot_description(motors: dict, arm: dict) -> str:
    shoulder_motor_sign = arm.get("shoulder_motor_sign", arm.get("joint0_sign", 1.0))
    elbow_motor_sign = arm.get("elbow_motor_sign", arm.get("joint1_sign", 1.0))
    shoulder_zero_offset_rad = arm.get("shoulder_zero_offset_rad", 0.0)
    elbow_zero_offset_rad = arm.get("elbow_zero_offset_rad", 0.0)
    shoulder_mit_kp = arm.get("kp_joint0", 0.0)
    shoulder_mit_kd = arm.get("kd_joint0", 0.0)
    shoulder_mit_ff = arm.get("feedforward_joint0", 0.0)
    elbow_mit_kp = arm.get("kp_joint1", 0.0)
    elbow_mit_kd = arm.get("kd_joint1", 0.0)
    elbow_mit_ff = arm.get("feedforward_joint1", 0.0)
    elbow_rel_min = arm.get("elbow_rel_min", -1.3962634)
    elbow_rel_max = arm.get("elbow_rel_max", 1.3962634)

    return textwrap.dedent(
        f"""\
        <?xml version="1.0"?>
        <robot name="r2_arm">
          <link name="base_link">
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="0.12 0.12 0.12"/>
              </geometry>
              <material name="gray">
                <color rgba="0.4 0.4 0.4 1.0"/>
              </material>
            </visual>
            <collision>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="0.12 0.12 0.12"/>
              </geometry>
            </collision>
          </link>

          <link name="upper_arm_link">
            <visual>
              <origin xyz="0 0 {arm['d1'] / 2.0}" rpy="0 0 0"/>
              <geometry>
                <box size="0.04 0.04 {arm['d1']}"/>
              </geometry>
              <material name="orange">
                <color rgba="0.9 0.6 0.2 1.0"/>
              </material>
            </visual>
            <collision>
              <origin xyz="0 0 {arm['d1'] / 2.0}" rpy="0 0 0"/>
              <geometry>
                <box size="0.04 0.04 {arm['d1']}"/>
              </geometry>
            </collision>
          </link>

          <link name="forearm_link">
            <visual>
              <origin xyz="{arm['d2'] / 2.0} 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{arm['d2']} 0.03 0.03"/>
              </geometry>
              <material name="blue">
                <color rgba="0.2 0.4 0.9 1.0"/>
              </material>
            </visual>
            <collision>
              <origin xyz="{arm['d2'] / 2.0} 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{arm['d2']} 0.03 0.03"/>
              </geometry>
            </collision>
          </link>

          <link name="tool_link">
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <sphere radius="0.02"/>
              </geometry>
              <material name="red">
                <color rgba="0.9 0.1 0.1 1.0"/>
              </material>
            </visual>
            <collision>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <sphere radius="0.02"/>
              </geometry>
            </collision>
          </link>

          <joint name="shoulder_joint" type="revolute">
            <parent link="base_link"/>
            <child link="upper_arm_link"/>
            <origin xyz="0 0 0.06" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="{arm['q1_min']}" upper="{arm['q1_max']}" effort="30.0" velocity="2.0"/>
          </joint>

          <joint name="elbow_joint" type="revolute">
            <parent link="upper_arm_link"/>
            <child link="forearm_link"/>
            <origin xyz="0 0 {arm['d1']}" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="{elbow_rel_min}" upper="{elbow_rel_max}" effort="30.0" velocity="2.0"/>
          </joint>

          <joint name="tool_joint" type="fixed">
            <parent link="forearm_link"/>
            <child link="tool_link"/>
            <origin xyz="{arm['d2']} 0 0" rpy="0 0 0"/>
          </joint>

          <ros2_control name="R2ArmSystem" type="system">
            <hardware>
              <plugin>r2_arm_control/DmHW</plugin>
              <param name="feedback_timeout_sec">{arm.get('feedback_timeout_sec', 0.05)}</param>
              <param name="feedback_hard_timeout_sec">{arm.get('feedback_hard_timeout_sec', 0.30)}</param>
              <param name="feedback_startup_grace_sec">{arm.get('feedback_startup_grace_sec', 0.0)}</param>
              <param name="read_error_log_interval_sec">{arm.get('read_error_log_interval_sec', 0.50)}</param>
              <param name="feedback_status_topic">{escape(str(arm.get('feedback_status_topic', '/r2/arm/feedback_status')))}</param>
              <param name="mit_position_command_alpha">{arm.get('mit_position_command_alpha', 1.0)}</param>
              <param name="mit_velocity_command_alpha">{arm.get('mit_velocity_command_alpha', 1.0)}</param>
              <param name="mit_effort_command_alpha">{arm.get('mit_effort_command_alpha', 1.0)}</param>
              <param name="mit_gain_command_alpha">{arm.get('mit_gain_command_alpha', 1.0)}</param>
            </hardware>

            <joint name="shoulder_joint">
              <param name="serial_port">{escape(str(motors['joint0']['serial_port']))}</param>
              <param name="baud_rate">{motors['joint0']['baud_rate']}</param>
              <param name="can_id">{motors['joint0']['can_id']}</param>
              <param name="mst_id">{motors['joint0']['mst_id']}</param>
              <param name="motor_type">{escape(str(motors['joint0']['motor_type']))}</param>
              <param name="motor_sign">{shoulder_motor_sign}</param>
              <param name="zero_offset_rad">{shoulder_zero_offset_rad}</param>
              <param name="position_min">{arm['q1_min']}</param>
              <param name="position_max">{arm['q1_max']}</param>
              <param name="mit_kp">{shoulder_mit_kp}</param>
              <param name="mit_kd">{shoulder_mit_kd}</param>
              <param name="mit_feedforward">{shoulder_mit_ff}</param>
              <command_interface name="position"/>
              <command_interface name="velocity"/>
              <command_interface name="effort"/>
              <command_interface name="kp"/>
              <command_interface name="kd"/>
              <state_interface name="position"/>
              <state_interface name="velocity"/>
              <state_interface name="effort"/>
            </joint>

            <joint name="elbow_joint">
              <param name="serial_port">{escape(str(motors['joint1']['serial_port']))}</param>
              <param name="baud_rate">{motors['joint1']['baud_rate']}</param>
              <param name="can_id">{motors['joint1']['can_id']}</param>
              <param name="mst_id">{motors['joint1']['mst_id']}</param>
              <param name="motor_type">{escape(str(motors['joint1']['motor_type']))}</param>
              <param name="motor_sign">{elbow_motor_sign}</param>
              <param name="zero_offset_rad">{elbow_zero_offset_rad}</param>
              <param name="position_min">{elbow_rel_min}</param>
              <param name="position_max">{elbow_rel_max}</param>
              <param name="mit_kp">{elbow_mit_kp}</param>
              <param name="mit_kd">{elbow_mit_kd}</param>
              <param name="mit_feedforward">{elbow_mit_ff}</param>
              <command_interface name="position"/>
              <command_interface name="velocity"/>
              <command_interface name="effort"/>
              <command_interface name="kp"/>
              <command_interface name="kd"/>
              <state_interface name="position"/>
              <state_interface name="velocity"/>
              <state_interface name="effort"/>
            </joint>
          </ros2_control>
        </robot>
        """
    )


def _launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    share_dir = get_package_share_directory("r2_arm_control")
    motors_yaml = LaunchConfiguration("motors_yaml").perform(context)
    arm_yaml = LaunchConfiguration("arm_yaml").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)

    motors_config = _load_yaml(motors_yaml)
    arm_config = _load_yaml(arm_yaml)
    controllers_yaml = os.path.join(share_dir, "config", "controllers.yaml")
    urdf_path = os.path.join(share_dir, "urdf", "r2_arm.urdf.xacro")
    actions = []

    xacro_executable = shutil.which("xacro")
    if xacro_executable:
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
    else:
        actions.append(
            LogInfo(
                msg="xacro executable was not found. Falling back to the launch-side URDF generator."
            )
        )
        robot_description_content = _build_robot_description(motors_config, arm_config)
    robot_description = {"robot_description": robot_description_content}

    ros2_control_enabled = IfCondition(LaunchConfiguration("start_ros2_control"))
    if "joint0_sign" in arm_config and "shoulder_motor_sign" not in arm_config:
        actions.append(
            LogInfo(
                msg="Deprecated parameter 'joint0_sign' detected. Use 'shoulder_motor_sign' instead."
            )
        )
    if "joint1_sign" in arm_config and "elbow_motor_sign" not in arm_config:
        actions.append(
            LogInfo(
                msg="Deprecated parameter 'joint1_sign' detected. Use 'elbow_motor_sign' instead."
            )
        )
    actions.append(
        LogInfo(
            msg="start_command_server is deprecated in bringup.launch.py. Launch r2_arm_moveit_config/launch/moveit.launch.py for MoveIt service execution.",
            condition=IfCondition(LaunchConfiguration("start_command_server")),
        )
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        condition=ros2_control_enabled,
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=ros2_control_enabled,
        output="screen",
    )

    mit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_mit_controller", "--controller-manager", "/controller_manager"],
        condition=ros2_control_enabled,
        output="screen",
    )

    arm_pose_test_node = Node(
        package="r2_arm_control",
        executable="arm_pose_test_node",
        parameters=[
            arm_config,
            {
                "q1_phys_deg": LaunchConfiguration("pose_q1_phys_deg"),
                "q2_phys_deg": LaunchConfiguration("pose_q2_phys_deg"),
                "use_target_ik": LaunchConfiguration("pose_use_target_ik"),
                "target_x": LaunchConfiguration("pose_target_x"),
                "target_z": LaunchConfiguration("pose_target_z"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_pose_test_node")),
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        output="screen",
    )

    actions.extend(
        [
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner,
            mit_controller_spawner,
            arm_pose_test_node,
            rviz_node,
        ]
    )
    return actions


def generate_launch_description():
    share_dir = get_package_share_directory("r2_arm_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motors_yaml",
                default_value=os.path.join(share_dir, "config", "motors.yaml"),
            ),
            DeclareLaunchArgument(
                "arm_yaml",
                default_value=os.path.join(share_dir, "config", "arm.yaml"),
            ),
            DeclareLaunchArgument(
                "start_ros2_control",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "start_command_server",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "start_pose_test_node",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "pose_q1_phys_deg",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "pose_q2_phys_deg",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "pose_use_target_ik",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "pose_target_x",
                default_value="0.30",
            ),
            DeclareLaunchArgument(
                "pose_target_z",
                default_value="0.30",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(share_dir, "rviz", "r2_arm.rviz"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
