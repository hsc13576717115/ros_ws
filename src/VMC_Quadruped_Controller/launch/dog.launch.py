import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu_config = os.path.join(
        get_package_share_directory("yesense_std_ros2"),
        "config",
        "yesense_config.yaml",
    )
    arm_system_launch = os.path.join(
        get_package_share_directory("r2_arm_control"),
        "launch",
        "system.launch.py",
    )
    arm_yaml = os.path.join(
        get_package_share_directory("r2_arm_control"),
        "config",
        "arm.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_arm", default_value="true"),
            DeclareLaunchArgument(
                "arm_start_rviz",
                default_value="false",
                description="Start the arm MoveIt RViz session.",
            ),
            DeclareLaunchArgument(
                "start_dpad_gpio",
                default_value="true",
                description="Start the integrated arm state machine / GPIO workflow.",
            ),
            DeclareLaunchArgument(
                "start_usb_imu",
                default_value="true",
                description="Start the USB Yesense IMU for manual VMC mode.",
            ),
            Node(
                package="joy",
                executable="joy_node",
                parameters=[{"autorepeat_rate": 20.0}],
                output="screen",
            ),
            Node(
                package="yesense_std_ros2",
                executable="yesense_node_publisher",
                name="yesense_pub",
                parameters=[imu_config],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_usb_imu")),
            ),
            Node(
                package="vmc_quadruped_controller",
                executable="foots",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(arm_system_launch),
                launch_arguments={
                    "start_moveit": LaunchConfiguration("start_arm"),
                    "start_state_machine": LaunchConfiguration("start_dpad_gpio"),
                    "start_rviz": LaunchConfiguration("arm_start_rviz"),
                    "arm_yaml": arm_yaml,
                }.items(),
            ),
        ]
    )
