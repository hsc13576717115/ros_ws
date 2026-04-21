import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}
 
def generate_launch_description():

    imu_config = os.path.join(
        get_package_share_directory('yesense_std_ros2'),
        'config',
        'yesense_config.yaml',
    )
    arm_moveit_launch = os.path.join(
        get_package_share_directory('r2_arm_moveit_config'),
        'launch',
        'moveit.launch.py',
    )
    arm_yaml = os.path.join(
        get_package_share_directory('r2_arm_control'),
        'config',
        'arm.yaml',
    )
    arm_config = _load_yaml(arm_yaml)

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_arm',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_dpad_gpio',
            default_value='true',
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{
                'autorepeat_rate': 0.0,
            }]
        ),
        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            parameters=[imu_config],
            output='screen',
        ),
        Node(
            package='vmc_quadruped_controller',
            executable='foots',
            output='screen'
        ),
        Node(
            package='r2_arm_control',
            executable='arm_state_machine_node.py',
            output='log',
            parameters=[arm_config],
            condition=IfCondition(LaunchConfiguration('start_dpad_gpio')),
        ),
        # Node(
        #     package='vmc_quadruped_controller',
        #     executable='navigator',
        #     output='log'
        # )
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_moveit_launch),
            launch_arguments={
                'arm_yaml': arm_yaml,
                'start_rviz': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_arm')),
        ),
    ])
