# yesense_ros2 stack

这是 Yesense 驱动源码栈的根目录，不是单一 ROS 包。当前目录下主要包含两个实际包：

- [`yesense_interface`](yesense_interface/README.md)：自定义消息接口
- [`yesense_std_ros2`](yesense_std_ros2/README.md)：Yesense IMU/GNSS 驱动

## 在工作区中的角色

- 为兼容链路 `N10P + Yesense + Cartographer` 提供 IMU 输入
- 也可单独给四足底盘提供姿态数据
- 当前默认配置会把 ROS 标准 IMU 数据发布到 `/imu/data_raw`

## 当前常用入口

```bash
ros2 launch yesense_std_ros2 yesense_node.launch.py
```

## 推荐阅读顺序

1. 先看 [`yesense_std_ros2/README.md`](yesense_std_ros2/README.md)
2. 需要理解消息结构时再看 [`yesense_interface/README.md`](yesense_interface/README.md)
