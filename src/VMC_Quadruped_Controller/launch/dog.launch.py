from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():

    imu_config = os.path.join(
        get_package_share_directory('yesense_std_ros2'),
        'config',
        'yesense_config.yaml',
    )
    arm_bringup_launch = os.path.join(
        get_package_share_directory('r2_arm_control'),
        'launch',
        'bringup.launch.py',
    )

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
            executable='dpad_gpio_toggle_node.py',
            output='screen',
            parameters=[{
                'gpio_number': 36,
            }],
            condition=IfCondition(LaunchConfiguration('start_dpad_gpio')),
        ),
        # Node(
        #     package='vmc_quadruped_controller',
        #     executable='navigator',
        #     output='log'
        # )
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_bringup_launch),
            launch_arguments={
                'start_ros2_control': 'true',
                'start_command_server': 'true',
                'start_rviz': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_arm')),
        ),
    ])
