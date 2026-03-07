# YOLOv11n RKNN 检测包使用指南

RK3588 平台 YOLOv11n 目标检测完整解决方案 - 从 ONNX 模型转换到 ROS2 部署

---

## 目录

1. [包概述](#包概述)
2. [环境准备](#环境准备)
3. [目录结构](#目录结构)
4. [快速开始](#快速开始)
5. [性能优化](#性能优化) ⭐
6. [模型转换与量化](#模型转换与量化)
7. [部署运行](#部署运行)
8. [跨工作空间复用](#跨工作空间复用)
9. [故障排查](#故障排查)

---

## 包概述

`yolov11n_rknn` 是一个完整的 ROS2 Humble 目标检测解决方案，具有以下特点：

- ✅ **ONNX → RKNN 模型转换** - 完整的转换流程
- ✅ **INT8 量化校准** - 自动化数据集生成
- ✅ **RK3588 NPU 硬件加速** - 6 TOPS 算力
- ✅ **高性能检测** - 30-31 FPS（无显示模式）
- ✅ **完全自包含** - 所有依赖内置，易于移植
- ✅ **跨工作空间复用** - 相对路径设计，开箱即用

### 技术规格

| 项目 | 规格 |
|------|------|
| **目标平台** | RK3588 (OrangePi 5 Plus) |
| **ROS2 版本** | Humble |
| **输入分辨率** | 640x640 |
| **摄像头采集** | 640x480 @ 120fps (MJPG) |
| **模型格式** | RKNN (INT8 量化) |
| **NPU 频率** | 1000 MHz |
| **典型性能** | 28-31 FPS |

---

## 环境准备

### 系统要求

- Ubuntu 22.04 (Jammy)
- ROS2 Humble
- RK3588 平台

### 安装依赖

```bash
# ROS2 依赖
sudo apt install -y \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-ament-index-cpp \
    ros-humble-image-transport

sudo apt-get install -y ros-humble-vision-msgs
# 系统依赖
sudo apt install -y \
    libopencv-dev

# Python 依赖（模型转换）
pip3 install rknn-toolkit2 opencv-python numpy ament-index-python
```

---

## 目录结构

```
yolov11n_rknn/
├── CMakeLists.txt                 # 编译配置
├── package.xml                    # 包描述
├── README.md                      # 项目说明
├── USAGE_GUIDE.md                 # 本文档
├── QUICKREF.md                    # 快速参考
│
├── include/yolov11n_rknn/         # 头文件
│   ├── types.hpp                  # 数据结构定义
│   └── yolo_detector.hpp          # 检测器类声明
│
├── src/                           # 源代码
│   ├── yolo_detection_node.cpp    # ROS2 节点
│   └── yolo_detector.cpp          # 检测器实现
│
├── launch/                        # 启动文件
│   └── yolo_detection.launch.py   # 主启动文件
│
├── config/                        # 配置文件
│   └── detection_params.yaml      # 检测参数
│
├── models/                        # 模型文件目录（相对路径）
│   ├── light.rknn                 # RKNN 模型
│   ├── light.onnx                 # ONNX 源模型
│   ├── dataset.txt                # 量化校准数据集
│   └── test_result.jpg            # 转换测试结果
│
├── data/                          # 测试数据目录
│   └── *.jpg                     # 校准图像
│
├── rknn-toolkit2/                 # RKNN SDK（本地）⭐
│   └── rknpu2/runtime/Linux/librknn_api/
│       ├── include/rknn_api.h     # 头文件
│       └── aarch64/librknnrt.so   # 运行时库 (v2.3.0)
│
├── scripts/                       # 工具脚本 ⭐
│   ├── diagnose_performance.sh     # 性能诊断
│   ├── optimize_performance.sh     # 临时性能优化
│   └── setup_performance_permanent.sh  # 永久性能优化
│
├── onnx2rknn_zq.py               # ONNX→RKNN 转换脚本
└── update_dataset.py              # 数据集生成脚本
```

### ⭐ 关键设计特点

**1. 自包含 RKNN SDK**
- 所有 RKNN 依赖内置在 `rknn-toolkit2/` 目录
- 无需系统安装 RKNN 库
- 版本锁定为 v2.3.0

**2. 相对路径设计**
- 所有路径使用 `ament_index` 自动解析
- 模型文件相对于 `share/yolov11n_rknn/`
- 支持跨工作空间复用

**3. 性能优化脚本**
- 一键设置 CPU/NPU 为性能模式
- 支持临时优化和永久优化
- 自动检测 NPU 设备

---

## 快速开始

### 1. 编译安装

```bash
cd ~/nav_ws

# 激活 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译
colcon build --packages-select yolov11n_rknn

# 激活工作空间
source install/setup.bash
```

### 2. 性能优化（必需！）⭐

**RK3588 性能优化对 FPS 影响巨大！**

```bash
# 方法 A：临时优化（重启后失效）
sudo bash ~/nav_ws/src/yolov11n_rknn/scripts/optimize_performance.sh

# 方法 B：永久优化（推荐）⭐
sudo bash ~/nav_ws/src/yolov11n_rknn/scripts/setup_performance_permanent.sh
```

**预期性能提升**：17 fps → 30-31 fps（+80%）

### 3. 运行检测

**带显示窗口（调试用）**：
```bash
ros2 launch yolov11n_rknn yolo_detection.launch.py
```

**无显示窗口（生产环境，推荐）**：
```bash
ros2 launch yolov11n_rknn yolo_detection.launch.py show_detection:=false
```

### 4. 查看检测结果

```bash
# 终端 1：运行节点（无显示模式）
ros2 launch yolov11n_rknn yolo_detection.launch.py show_detection:=false

# 终端 2：查看检测结果
ros2 topic echo /yolo/detections
```

---

## 性能优化 ⭐

### 为什么需要性能优化？

RK3588 默认使用节能模式，CPU 和 NPU 会动态降频，严重影响性能。

### 性能对比

| 配置 | FPS | 说明 |
|------|-----|------|
| **默认（节能模式）** | 17 fps | CPU/NPU 动态降频 |
| **性能模式 + 显示** | 22-25 fps | 窗口渲染消耗资源 |
| **性能模式 + 无显示** | **30-31 fps** ⭐ | **最佳性能** |

### 性能优化方法

#### 方法 1：临时优化（测试用）

```bash
sudo bash ~/nav_ws/src/yolov11n_rknn/scripts/optimize_performance.sh
```

**效果**：
- CPU governor: ondemand → performance
- NPU governor: rknpu_ondemand → performance
- NPU 频率: 固定 1000 MHz

**限制**：重启后失效

#### 方法 2：永久优化（推荐）⭐

```bash
sudo bash ~/nav_ws/src/yolov11n_rknn/scripts/setup_performance_permanent.sh
```

**效果**：
- 创建 systemd 服务
- 开机自动设置性能模式
- 重启后自动生效

**管理服务**：
```bash
# 查看服务状态
systemctl status rknn-performance

# 手动启动
systemctl start rknn-performance

# 禁用自动启动
systemctl disable rknn-performance
```

### 性能诊断

```bash
# 运行性能诊断
bash ~/nav_ws/src/yolov11n_rknn/scripts/diagnose_performance.sh
```

输出：
- CPU 频率模式
- NPU 频率模式
- 系统负载
- 内存使用
- 优化建议

### 性能监控

**无显示模式自动输出性能**：
```
[INFO] FPS: 30 | Cap: 5.7ms | Pre: 1.8ms | Inf: 20.5ms | Post: 0.4ms | Detect: 22.7ms | Det: 2
```

| 指标 | 说明 | 预期值 |
|------|------|--------|
| **Cap** | 摄像头采集时间 | 5-7ms |
| **Pre** | 预处理时间 | 1-4ms |
| **Inf** | NPU 推理时间 | 17-24ms |
| **Post** | 后处理时间 | 0.4-1.2ms |
| **Detect** | 检测总时间 | 20-30ms |
| **FPS** | 实际帧率 | 28-31 |

---

## 模型转换与量化

### 步骤 1：准备 ONNX 模型

将 YOLOv11 ONNX 模型放到 `models/` 目录：

```bash
cp /path/to/yolov11n.onnx ~/nav_ws/src/yolov11n_rknn/models/light.onnx
```

### 步骤 2：准备量化校准数据集

**自动生成（推荐）**：
```bash
cd ~/nav_ws/src/yolov11n_rknn

# 扫描 data/ 目录，生成绝对路径
python3 update_dataset.py

# 生成相对路径（跨工作空间）
python3 update_dataset.py --relative

# 随机选择 50 张图片
python3 update_dataset.py --shuffle --limit 50
```

**手动创建**：
```bash
cat > models/dataset.txt << EOF
data/image_0.jpg
data/image_1.jpg
data/image_2.jpg
EOF
```

**数据集建议**：
- 最少: 10-20 张（快速测试）
- 推荐: 50-200 张（良好精度）
- 图片应覆盖实际应用场景

### 步骤 3：配置转换参数

编辑 `onnx2rknn_zq.py`：

```python
# ========== 可配置参数 ==========

# 输入输出路径（自动解析）
ONNX_MODEL = os.path.join(MODELS_DIR, 'light.onnx')
RKNN_MODEL = os.path.join(MODELS_DIR, 'light.rknn')
DATASET = os.path.join(MODELS_DIR, 'dataset.txt')

# 量化配置
QUANTIZE_ON = True           # True=INT8量化, False=FP16

# 目标平台
TARGET_PLATFORM = 'rk3588'   # rk3588, rk3566, rk3568

# 模型配置
CLASSES = ['ball']           # 类别名称
class_num = len(CLASSES)

# 输入分辨率（固定 640x640）
input_imgH = 640
input_imgW = 640
mapSize = [[80, 80], [40, 40], [20, 20]]
```

### 步骤 4：运行转换

```bash
cd ~/nav_ws/src/yolov11n_rknn
python3 onnx2rknn_zq.py
```

**输出示例**：
```
======================================================================
YOLOv11 ONNX to RKNN Conversion Script
======================================================================
Package root: /home/orangepi/nav_ws/src/yolov11n_rknn
Models directory: /home/orangepi/nav_ws/src/yolov11n_rknn/models
Input resolution: 640x640
======================================================================

--> Config model
  Target platform: rk3588
  Quantization: Enabled

--> Loading ONNX model
  done

--> Building RKNN model
  Using dataset: models/dataset.txt
  Quantization: INT8
  done

--> Export RKNN model
  Output: models/light.rknn
  done

Conversion completed successfully!
======================================================================
```

### 步骤 5：验证转换结果

```bash
# 检查生成的模型
ls -lh models/light.rknn

# 查看测试结果图像
display models/test_result.jpg
```

---

## 部署运行

### 启动参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `show_detection` | bool | `true` | 是否显示检测窗口 |
| `publish_image` | bool | `false` | 是否发布检测图像 |
| `camera_id` | int | `0` | 摄像头设备 ID |
| `model_path` | string | `models/light.rknn` | 模型路径（相对） |
| `num_classes` | int | `1` | 类别数量 |
| `class_names` | string[] | `["ball"]` | 类别名称 |
| `conf_threshold` | double | `0.5` | 置信度阈值 |
| `nms_threshold` | double | `0.45` | NMS IoU 阈值 |

### 运行模式

#### 模式 1：调试模式（带显示）

```bash
ros2 launch yolov11n_rknn yolo_detection.launch.py
```

**特点**：
- 实时显示检测结果
- 显示 FPS 和推理时间
- FPS: 22-25（显示渲染消耗资源）

#### 模式 2：生产模式（无显示）⭐

```bash
ros2 launch yolov11n_rknn yolo_detection.launch.py show_detection:=false
```

**特点**：
- 无显示窗口
- 终端每秒输出性能统计
- FPS: 28-31（最佳性能）

**性能输出示例**：
```
[INFO] FPS: 30 | Cap: 5.7ms | Pre: 1.8ms | Inf: 20.5ms | Post: 0.4ms | Detect: 22.7ms | Det: 2
```

#### 模式 3：自定义参数

```bash
# 自定义阈值
ros2 launch yolov11n_rknn yolo_detection.launch.py conf_threshold:=0.6

# 多类别
ros2 launch yolov11n_rknn yolo_detection.launch.py \
    num_classes:=3 \
    class_names:=["cat","dog","bird"]

# 指定摄像头
ros2 launch yolov11n_rknn yolo_detection.launch.py camera_id:=1
```

### ROS2 话题接口

#### 发布的话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | 检测结果 |
| `/yolo/detection_image` | `sensor_msgs/msg/Image` | 带检测框的图像（可选） |

#### 查看检测结果

```bash
# 实时查看
ros2 topic echo /yolo/detections

# 查看话题信息
ros2 topic info /yolo/detections

# 查看发布频率
ros2 topic hz /yolo/detections
```

#### 消息格式

```yaml
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: 'camera'
detections:
- header:
    stamp: {sec: 1234567890, nanosec: 123456789}
    frame_id: 'camera'
  results:
  - hypothesis:
      class_id: 'ball'
      score: 0.85
    bbox:
      center:
        position: {x: 320.0, y: 240.0}
        theta: 0.0
      size_x: 100.0
      size_y: 100.0
```

---

## 跨工作空间复用

本包设计为**完全自包含**，可轻松复制到任何 ROS2 工作空间。

### 复制到新工作空间

```bash
# 1. 复制整个包
cp -r ~/nav_ws/src/yolov11n_rknn /new_ws/src/

# 2. 在新工作空间编译
cd /new_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select yolov11n_rknn
source install/setup.bash

# 3. 运行（所有路径自动解析）
ros2 launch yolov11n_rknn yolo_detection.launch.py
```

### 路径解析机制

| 组件 | 路径解析方式 |
|------|-------------|
| **C++ 节点** | `ament_index_cpp::get_package_share_directory("yolov11n_rknn")` |
| **Python 启动** | `get_package_share_directory('yolov11n_rknn')` |
| **转换脚本** | 自动检测包安装目录或源码目录 |
| **模型文件** | 相对于 `share/yolov11n_rknn/` |

### 数据集注意事项

**⚠️ `models/dataset.txt` 可能需要重新生成**

如果使用了绝对路径，复制到新工作空间后需要重新生成：

```bash
cd /new_ws/src/yolov11n_rknn
python3 update_dataset.py --relative
```

### 自包含性说明

**完全自包含**（无需额外安装）：
- ✅ RKNN SDK（12MB）
- ✅ RKNN 运行时库（7MB）
- ✅ RKNN 头文件
- ✅ 所有工具脚本

**外部依赖**（标准系统包）：
- ROS2 Humble（rclcpp, vision_msgs, cv_bridge等）
- OpenCV 4
- Python 3 + rknn-toolkit2（仅转换时需要）

---

## 故障排查

### 问题 1：FPS 低（< 25 fps）

**症状**：FPS 只有 17-22

**解决方案**：
```bash
# 1. 检查当前性能模式
bash ~/nav_ws/src/yolov11n_rknn/scripts/diagnose_performance.sh

# 2. 设置性能模式
sudo bash ~/nav_ws/src/yolov11n_rknn/scripts/optimize_performance.sh

# 3. 重启节点
ros2 launch yolov11n_rknn yolo_detection.launch.py show_detection:=false
```

**预期**：FPS 提升到 28-31

### 问题 2：模型加载失败

**错误信息**：`Failed to load RKNN model`

**解决方案**：
```bash
# 1. 检查模型文件是否存在
ls install/yolov11n_rknn/share/yolov11n_rknn/models/light.rknn

# 2. 查看包安装路径
ros2 pkg prefix yolov11n_rknn

# 3. 重新转换模型
cd ~/nav_ws/src/yolov11n_rknn
python3 onnx2rknn_zq.py

# 4. 重新编译安装
colcon build --packages-select yolov11n_rknn --symlink-install
```

### 问题 3：摄像头无法打开

**错误信息**：`Failed to open camera`

**解决方案**：
```bash
# 1. 检查摄像头设备
ls -l /dev/video*

# 2. 查看摄像头详情
v4l2-ctl --list-devices

# 3. 修改 camera_id 参数
ros2 launch yolov11n_rknn yolo_detection.launch.py camera_id:=1
```

### 问题 4：检测结果为空

**解决方案**：
```bash
# 1. 降低置信度阈值
ros2 launch yolov11n_rknn yolo_detection.launch.py conf_threshold:=0.2

# 2. 检查摄像头画面
ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 test.jpg

# 3. 确认模型训练类别正确
ros2 launch yolov11n_rknn yolo_detection.launch.py \
    num_classes:=1 \
    class_names:=["your_class"]
```

### 问题 5：转换时量化数据集错误

**错误信息**：`The image of xxx is invalid!`

**解决方案**：
```bash
# 1. 检查 dataset.txt 格式（不能有注释行）
cat models/dataset.txt

# 2. 使用相对路径重新生成
cd ~/nav_ws/src/yolov11n_rknn
python3 update_dataset.py --relative

# 3. 确保图像文件存在
ls -l $(cat models/dataset.txt | head -5)
```

### 问题 6：编译错误找不到 rknn_api.h

**解决方案**：
```bash
# 1. 检查头文件是否存在
ls rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include/rknn_api.h

# 2. 清理重新编译
cd build/yolov11n_rknn
rm -rf *
cd ~/nav_ws
colcon build --packages-select yolov11n_rknn
```

### 问题 7：显示窗口卡顿

**解决方案**：
```bash
# 使用无显示模式
ros2 launch yolov11n_rknn yolo_detection.launch.py show_detection:=false
```

**原因**：OpenCV 窗口渲染消耗 10-15ms，导致 FPS 从 30 降到 22。

---

## 附录

### 完整参数列表

| 参数 | 类型 | 默认值 | 范围 | 说明 |
|------|------|--------|------|------|
| `model_path` | string | `"models/light.rknn"` | - | RKNN 模型路径（相对） |
| `num_classes` | int | `1` | 1-N | 类别数量 |
| `class_names` | string[] | `["ball"]` | - | 类别名称列表 |
| `conf_threshold` | double | `0.5` | 0.0-1.0 | 置信度阈值 |
| `nms_threshold` | double | `0.45` | 0.0-1.0 | NMS IoU 阈值 |
| `show_detection` | bool | `true` | - | 是否显示检测窗口 |
| `publish_image` | bool | `false` | - | 是否发布检测图像 |
| `camera_id` | int | `0` | 0-N | 摄像头设备 ID |
| `input_width` | int | `640` | - | 输入宽度（固定） |
| `input_height` | int | `640` | - | 输入高度（固定） |

### 参考资料

- [RKNN Toolkit2 文档](https://github.com/airockchip/rknn-toolkit2)
- [RKNPU2 SDK](https://github.com/rockchip-linux/rknpu2)
- [YOLOv11 官方仓库](https://github.com/ultralytics/ultralytics)
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)

### 性能基准

**测试环境**：
- 平台：OrangePi 5 Plus (RK3588)
- CPU/NPU：performance 模式
- 摄像头：USB MJPG 640x480@120fps
- 模型：YOLOv11n INT8

**测试结果**：

| 模式 | FPS | 推理时间 | CPU 使用率 |
|------|-----|----------|-----------|
| 带显示 | 22-25 | 17-24ms | 65-75% |
| **无显示** | **28-31** | **17-24ms** | **55-65%** |

### 版本历史

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| 1.0.0 | 2026-03-03 | 初始版本 |
| 1.1.0 | 2026-03-03 | 添加性能优化脚本 |
| 1.2.0 | 2026-03-03 | 优化摄像头配置，更新文档 |

---

**版本**: 1.2.0
**更新日期**: 2026-03-03
**许可证**: Apache-2.0
**维护者**: orangepi
