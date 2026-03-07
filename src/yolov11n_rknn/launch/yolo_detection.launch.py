#!/usr/bin/env python3
"""
YOLOv11n RKNN Detection Launch File

使用示例:
    # 基本启动（显示检测窗口）
    ros2 launch yolov11n_rknn yolo_detection.launch.py

    # 自定义参数启动
    ros2 launch yolov11n_rknn yolo_detection.launch.py conf_threshold:=0.6

    # 启动并发布检测图像
    ros2 launch yolov11n_rknn yolo_detection.launch.py publish_image:=true

    # 使用自定义模型
    ros2 launch yolov11n_rknn yolo_detection.launch.py model_path:=/path/to/model.rknn
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包共享目录
    pkg_share_dir = get_package_share_directory('yolov11n_rknn')

    # ---------------------------------------------------------------------
    # 声明启动参数
    # ---------------------------------------------------------------------
    # 模型相关
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([pkg_share_dir, 'models', 'light.rknn']),
        description='Path to RKNN model file (relative to package share directory)'
    )

    num_classes_arg = DeclareLaunchArgument(
        'num_classes',
        default_value='1',
        description='Number of object classes in the model'
    )

    class_names_arg = DeclareLaunchArgument(
        'class_names',
        default_value="['ball']",
        description='List of class names'
    )

    # 检测参数
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Confidence threshold for detection (0.0 - 1.0)'
    )

    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.45',
        description='NMS IoU threshold (0.0 - 1.0)'
    )

    # 可视化参数
    show_detection_arg = DeclareLaunchArgument(
        'show_detection',
        default_value='true',
        description='Show detection result window'
    )

    publish_image_arg = DeclareLaunchArgument(
        'publish_image',
        default_value='false',
        description='Publish detection image with bounding boxes'
    )

    # 摄像头参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID (usually 0 for /dev/video0)'
    )

    # ---------------------------------------------------------------------
    # YOLO 检测节点
    # ---------------------------------------------------------------------
    yolo_detection_node = Node(
        package='yolov11n_rknn',
        executable='yolo_detection_node',
        name='yolo_detection_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'num_classes': LaunchConfiguration('num_classes'),
            'class_names': LaunchConfiguration('class_names'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'show_detection': LaunchConfiguration('show_detection'),
            'publish_image': LaunchConfiguration('publish_image'),
            'camera_id': LaunchConfiguration('camera_id'),
        }]
    )

    # ---------------------------------------------------------------------
    # 返回启动描述
    # ---------------------------------------------------------------------
    return LaunchDescription([
        # 参数声明
        model_path_arg,
        num_classes_arg,
        class_names_arg,
        conf_threshold_arg,
        nms_threshold_arg,
        show_detection_arg,
        publish_image_arg,
        camera_id_arg,
        # 节点
        yolo_detection_node,
    ])
