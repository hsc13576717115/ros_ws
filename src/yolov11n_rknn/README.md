# yolov11n_rknn

基于 RKNN 的轻量级目标检测包，面向 OrangePi 等 ARM/NPU 平台。它不是导航必须项，但已经和 `nav2_config` 做了可选联动，可在导航任务运行时按需启停。

## 作用

- 读取摄像头图像
- 运行 `YOLOv11n` 检测
- 发布检测结果与可选可视化图像
- 通过 `/yolo/enable` 支持运行时启停

## 主要入口

- 节点：`yolo_detection_node`
- 启动文件：[`launch/yolo_detection.launch.py`](launch/yolo_detection.launch.py)
- 参数文件：[`config/detection_params.yaml`](config/detection_params.yaml)
- 默认模型目录：`models/`

## 常用命令

直接运行检测：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash

ros2 launch yolov11n_rknn yolo_detection.launch.py
```

不弹 OpenCV 窗口，但发布检测图像：

```bash
ros2 launch yolov11n_rknn yolo_detection.launch.py \
  show_detection:=false \
  publish_image:=true
```

在当前整机主链路里启用或关闭它：

```bash
ros2 launch nav2_config dog_slam_navigation.py start_yolo:=true
ros2 launch nav2_config dog_slam_navigation.py start_yolo:=false
```

## 关键话题

- `/yolo/detections`
- `/yolo/detection_image`
- `/yolo/enable`

## 常用参数

- `model_path`
- `conf_threshold`
- `nms_threshold`
- `show_detection`
- `publish_image`
- `camera_id`
- `start_enabled`
- `enable_topic`

## 备注

- 如果现场没有相机或 NPU 还没配好，建议先把 `start_yolo:=false`
- 这个包更多是“可选感知能力”，不是当前导航闭环的必要前提
