# YOLOv11n RKNN Detection Package

基于 RKNN 加速的 YOLOv11n 目标检测 ROS2 包，专为 OrangePi 等 ARM 平台优化。

## 📖 快速开始

> **完整使用指南**: 查看 [USAGE_GUIDE.md](USAGE_GUIDE.md) 获取从模型转换到部署的完整教程

```bash
# 1. 编译安装
cd /home/orangepi/nav_ws
colcon build --packages-select yolov11n_rknn
source install/setup.bash

# 2. 运行检测
ros2 launch yolov11n_rknn yolo_detection.launch.py
```

## 功能特性

- ✅ YOLOv11n 目标检测（RKNN 加速）
- ✅ 实时视频流处理
- ✅ 检测结果话题输出
- ✅ 可视化显示
- ✅ 可选的检测图像发布
- ✅ 模块化设计，易于扩展
- ✅ 支持跨工作空间复用（相对路径）

## 目录结构

```
yolov11n_rknn/
├── models/                          # 模型文件目录
│   ├── light-1.rknn                 # RKNN 模型（运行时使用）
│   ├── light.onnx                  # ONNX 模型（用于转换）
│   ├── dataset.txt                 # 量化校准数据集列表
│   └── test_result.jpg             # 转换测试结果图
├── data/                            # 测试数据目录
│   └── test.jpg                    # 测试图像
├── include/yolov11n_rknn/
│   ├── types.hpp                    # 数据结构定义
│   └── yolo_detector.hpp            # 检测器类声明
├── src/
│   ├── yolo_detector.cpp            # 检测器实现
│   └── yolo_detection_node.cpp      # ROS2节点
├── launch/
│   └── yolo_detection.launch.py     # 启动文件
├── config/
│   └── detection_params.yaml        # 参数配置
├── onnx2rknn_zq.py                  # 模型转换脚本
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 依赖项

### ROS2 依赖
```bash
sudo apt install -y \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-ament-index-cpp
```

### 系统依赖
```bash
sudo apt install -y \
    libopencv-dev \
    librknn-api-dev
```

## 编译

```bash
cd /home/orangepi/nav_ws
colcon build --packages-select yolov11n_rknn
source install/setup.bash
```

## 使用方法

### 1. 基本启动（显示检测窗口）

```bash
ros2 launch yolov11n_rknn yolo_detection.launch.py
```

### 2. 自定义参数启动

```bash
ros2 launch yolov11n_rknn yolo_detection.launch.py \
    conf_threshold:=0.6 \
    show_detection:=true
```

### 3. 多类别模型示例

```bash
ros2 launch yolov11n_rknn yolo_detection.launch.py \
    num_classes:=3 \
    class_names:=["ball", "cup", "person"]
```

### 4. 使用自定义模型

```bash
# 方法1: 使用绝对路径
ros2 launch yolov11n_rknn yolo_detection.launch.py \
    model_path:=/path/to/your/model.rknn

# 方法2: 将模型复制到包的 models 目录
cp your_model.rknn /path/to/yolov11n_rknn/models/light-1.rknn
ros2 launch yolov11n_rknn yolo_detection.launch.py
```

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | string | `models/light-1.rknn` | RKNN 模型路径（相对路径） |
| `num_classes` | int | `1` | 类别数量 |
| `class_names` | string[] | `["ball"]` | 类别名称列表 |
| `conf_threshold` | double | `0.5` | 置信度阈值 |
| `nms_threshold` | double | `0.45` | NMS IoU 阈值 |
| `show_detection` | bool | `true` | 是否显示检测窗口 |
| `publish_image` | bool | `false` | 是否发布检测图像 |
| `camera_id` | int | `0` | 摄像头设备 ID |

## 话题

### 发布的话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | 检测结果数组 |
| `/yolo/detection_image` | `sensor_msgs/msg/Image` | 带检测框的图像（可选） |

### 检测结果消息格式

```cpp
// vision_msgs/msg/Detection2DArray
std_msgs/Header header
Detection2D[] detections

// Detection2D
std_msgs/Header header
ObjectHypothesisWithPose[] results
BoundingBox2D bbox

// ObjectHypothesisWithPose
ObjectHypothesis hypothesis
Pose pose

// BoundingBox2D
float64 center_x
float64 center_y
float64 size_x
float64 size_y
```

## 模型管理

### 模型转换

将 ONNX 模型转换为 RKNN 格式：

```bash
cd /home/orangepi/nav_ws/src/yolov11n_rknn

# 1. 准备文件
# - 将 ONNX 模型放到 models/ 目录
# - 准备量化校准数据集列表 models/dataset.txt

# 2. 运行转换脚本
python3 onnx2rknn_zq.py
```

### 数据集格式

`models/dataset.txt` 格式：
```
/path/to/image1.jpg
/path/to/image2.jpg
/path/to/image3.jpg
...
```

### 目录说明

| 目录 | 用途 |
|------|------|
| `models/` | 存放所有模型相关文件（RKNN、ONNX、数据集） |
| `data/` | 存放测试图像 |
| `install/yolov11n_rknn/share/yolov11n_rknn/models/` | 安装后的模型路径 |

## 跨工作空间复用

此包使用相对路径，可以轻松复制到其他 ROS2 工作空间：

```bash
# 复制包到新工作空间
cp -r /path/to/yolov11n_rknn /new_ws/src/

# 在新工作空间中编译
cd /new_ws
colcon build --packages-select yolov11n_rknn
source install/setup.bash

# 运行（模型路径会自动解析）
ros2 launch yolov11n_rknn yolo_detection.launch.py
```

### 路径解析机制

- **C++ 节点**: 使用 `ament_index_cpp::get_package_share_directory()` 自动解析包路径
- **Python 启动文件**: 使用 `get_package_share_directory()` 获取包路径
- **转换脚本**: 自动检测包的安装目录或源码目录

## 代码示例

### C++ 订阅检测结果

```cpp
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

class DetectionSubscriber : public rclcpp::Node {
public:
    DetectionSubscriber() : Node("detection_subscriber") {
        subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/yolo/detections", 10,
            std::bind(&DetectionSubscriber::detection_callback, this, std::placeholders::_1)
        );
    }

private:
    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received %zu detections", msg->detections.size());

        for (const auto& detection : msg->detections) {
            const auto& bbox = detection.bbox;
            const auto& result = detection.results[0];

            RCLCPP_INFO(this->get_logger(),
                "Class: %s, Conf: %.2f, Center: (%.1f, %.1f), Size: (%.1f, %.1f)",
                result.hypothesis.class_id.c_str(),
                result.hypothesis.score,
                bbox.center.x, bbox.center.y,
                bbox.size_x, bbox.size_y
            );
        }
    }

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
};
```

### Python 订阅示例

```python
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.detections)} detections')

        for detection in msg.detections:
            bbox = detection.bbox
            if detection.results:
                result = detection.results[0]
                self.get_logger().info(
                    f'Class: {result.hypothesis.class_id}, '
                    f'Confidence: {result.hypothesis.score:.2f}, '
                    f'Center: ({bbox.center.x:.1f}, {bbox.center.y:.1f})'
                )


def main():
    rclpy.init()
    node = DetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 查看检测结果

```bash
# 查看话题
ros2 topic echo /yolo/detections

# 查看检测图像（如果启用）
ros2 run image_tools showimage --ros-args -r image_topic:=/yolo/detection_image

# 使用 rqt_image_view
ros2 run rqt_image_view rqt_image_view
```

## 故障排查

### 1. 摄像头无法打开
```bash
# 检查摄像头设备
ls -l /dev/video*

# 查看摄像头详情
v4l2-ctl --list-devices

# 修改 camera_id 参数
ros2 launch yolov11n_rknn yolo_detection.launch.py camera_id:=1
```

### 2. 模型加载失败
```bash
# 检查模型文件是否存在
ros2 run yolov11n_rknn yolo_detection_node.py --ros-args \
    -r model_path:=models/light-1.rknn

# 查看包安装路径
ros2 pkg prefix yolov11n_rknn
```

### 3. 检测结果为空
- 降低 `conf_threshold` 参数
- 检查摄像头画面是否正常
- 确认模型训练类别是否正确

### 4. 跨工作空间复用问题
```bash
# 确保包已在新工作空间中编译
colcon build --packages-select yolov11n_rknn

# 检查模型文件是否存在
ls install/yolov11n_rknn/share/yolov11n_rknn/models/

# 检查环境变量
echo $AMENT_PREFIX_PATH
```

## 代码架构

```
┌─────────────────────────────────────────────────────────────┐
│                    YoloDetectionNode                        │
│  - ROS2节点，管理摄像头、定时器、话题发布                    │
│  - 自动解析包路径，支持跨工作空间复用                            │
├─────────────────────────────────────────────────────────────┤
│                    YoloDetector                             │
│  - RKNN模型加载、推理、后处理                                │
│  - DFL解码、NMS、坐标转换                                    │
├─────────────────────────────────────────────────────────────┤
│                    DetectBox                                │
│  - 检测框数据结构                                            │
└─────────────────────────────────────────────────────────────┘
```

## 版本信息

- **输入分辨率**: 640x640
- **模型**: YOLOv11n
- **加速**: RKNN (INT8 量化)
- **平台**: RK3588 (OrangePi 5)

## 许可证

Apache-2.0
