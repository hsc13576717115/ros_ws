from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('yesense_std_ros2'),
        'config',
        'yesense_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to yesense configuration file',
        ),
        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),
    ])
