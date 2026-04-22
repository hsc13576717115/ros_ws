import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vmc_share = get_package_share_directory("vmc_quadruped_controller")
    nav_share = get_package_share_directory("nav2_config")

    dog_launch = os.path.join(vmc_share, "launch", "dog.launch.py")
    navigation_launch = os.path.join(nav_share, "launch", "dog_slam_navigation.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_yolo", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_arm_rviz", default_value="true"),
            DeclareLaunchArgument("start_preset_mission", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(dog_launch),
                launch_arguments={
                    "start_arm": "true",
                    "arm_start_rviz": LaunchConfiguration("start_arm_rviz"),
                    "start_dpad_gpio": "true",
                    "start_usb_imu": "false",
                }.items(),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(navigation_launch),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "start_yolo": LaunchConfiguration("start_yolo"),
                            "nav_start_rviz": LaunchConfiguration("start_rviz"),
                            "start_preset_mission": LaunchConfiguration(
                                "start_preset_mission"
                            ),
                        }.items(),
                    )
                ],
            ),
        ]
    )
