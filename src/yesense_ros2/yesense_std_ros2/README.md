# yesense_std_ros2

Yesense IMU/GNSS 驱动包。当前工作区里，它既可以给兼容 SLAM 链路提供 IMU 数据，也可以单独给四足底盘和姿态相关节点提供 `/imu/data_raw`。

## 作用

- 从串口读取 Yesense 设备数据
- 发布 ROS 标准 `sensor_msgs/msg/Imu`
- 同时发布 Yesense 自定义姿态/导航话题

## 主要入口

- 驱动节点：`yesense_node_publisher`
- 订阅示例：`yesense_node_subscriber`
- 启动文件：[`launch/yesense_node.launch.py`](launch/yesense_node.launch.py)
- 默认配置：[`config/yesense_config.yaml`](config/yesense_config.yaml)

## 当前默认配置

- 串口：`/dev/serial/by-id/usb-1a86_USB_Single_Serial_597B019761-if00`
- 波特率：`460800`
- `frame_id`：`gyro_link`
- ROS 标准 IMU 话题：`/imu/data_raw`
- Yesense 自定义 IMU 话题：`/imu_data`

## 常见输出话题

- `/imu/data_raw`
- `/imu_data`
- `/euler_only`
- `/att_min_vru`
- `/nav_min`

## 常用命令

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash

ros2 launch yesense_std_ros2 yesense_node.launch.py
```

查看数据是否正常：

```bash
ros2 topic echo /imu/data_raw --once
ros2 topic echo /euler_only --once
```

## 与其他包的关系

- 消息接口：[`yesense_interface`](../yesense_interface/README.md)
- 兼容 SLAM：[`slam_config`](../../slam_config/README.md)
- 四足底盘：[`vmc_quadruped_controller`](../../VMC_Quadruped_Controller/README.md)
