# YOLOv11n RKNN 快速参考

## 🚀 快速开始

```bash
# 编译
cd ~/nav_ws && colcon build --packages-select yolov11n_rknn && source install/setup.bash

# 运行
ros2 launch yolov11n_rknn yolo_detection.launch.py
```

## 🔄 模型转换

```bash
cd ~/nav_ws/src/yolov11n_rknn

# 1. 放置 ONNX 模型
cp your_model.onnx models/light.onnx

# 2. 编辑类别
# vim onnx2rknn_zq.py 修改 CLASSES = ['your', 'classes']

# 3. 自动生成数据集（默认绝对路径）
python3 update_dataset.py

# 或使用相对路径（跨工作空间）
python3 update_dataset.py --relative

# 或手动添加
echo "data/test.jpg" > models/dataset.txt

# 4. 转换
python3 onnx2rknn_zq.py
```

## 📡 常用命令

```bash
# 自定义参数
ros2 launch yolov11n_rknn yolo_detection.launch.py conf_threshold:=0.6

# 多类别
ros2 launch yolov11n_rknn yolo_detection.launch.py \
    num_classes:=3 class_names:=["cat","dog","bird"]

# 查看检测
ros2 topic echo /yolo/detections

# 查看图像
ros2 run rqt_image_view rqt_image_view
```

## 🔧 跨工作空间

```bash
# 复制包
cp -r ~/nav_ws/src/yolov11n_rknn /new_ws/src/

# 编译运行
cd /new_ws && colcon build --packages-select yolov11n_rknn && \
    source install/setup.bash && \
    ros2 launch yolov11n_rknn yolo_detection.launch.py
```

## 📂 目录结构

```
yolov11n_rknn/
├── models/              # 模型文件
├── data/                # 测试数据
├── include/             # 头文件
├── src/                 # 源代码
├── launch/              # 启动文件
├── config/              # 配置文件
├── onnx2rknn_zq.py      # 转换脚本
├── update_dataset.py    # 数据集生成脚本 🆕
├── USAGE_GUIDE.md       # 完整指南 ⭐
└── README.md            # 项目说明
```

## 🆘 故障排查

| 问题 | 解决方案 |
|------|----------|
| 摄像头打不开 | `camera_id:=1` |
| 模型加载失败 | 重新运行 `python3 onnx2rknn_zq.py` |
| 检测结果为空 | 降低 `conf_threshold:=0.2` |
| 找不到头文件 | 检查 `rknn-toolkit2/` 目录存在 |

---

📖 **详细指南**: [USAGE_GUIDE.md](USAGE_GUIDE.md)
