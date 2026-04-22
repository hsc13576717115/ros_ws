# lslidar_driver

Leishen 二维激光雷达驱动包，负责把 `M10 / N10 / N10P` 等设备接入 ROS 2。当前工作区里，它主要服务兼容链路 `N10P + Yesense + Cartographer + Nav2`。

## 作用

- 启动 Leishen 激光驱动节点 `lslidar_driver_node`
- 发布 `/scan`
- 可选发布点云 `/lslidar_point_cloud`
- 配合 `slam_config` 进入 Cartographer 建图

## 主要入口

- 可执行程序：`lslidar_driver_node`
- 常用串口配置：[`params/lidar_uart_ros2/lsn10p.yaml`](params/lidar_uart_ros2/lsn10p.yaml)
- 常用启动文件：
  - [`launch/lsn10p_launch.py`](launch/lsn10p_launch.py)
  - [`launch/lsn10p_net_launch.py`](launch/lsn10p_net_launch.py)
  - [`launch/lsm10_uart_launch.py`](launch/lsm10_uart_launch.py)
  - [`launch/viewer_scan_launch.py`](launch/viewer_scan_launch.py)

## 常用命令

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash

ros2 launch lslidar_driver lsn10p_launch.py
```

如果要走兼容的二维 SLAM 链路，直接启动：

```bash
ros2 launch slam_config slam_launch.py start_imu:=true
```

## 关键依赖

- [`lslidar_msgs`](../lslidar_msgs/README.md)
- `sensor_msgs`
- `pcl_conversions`

## 备注

- 当前工作区推荐导航主链路已经转向 `MID360 + FAST-LIO`
- 如果现场仍在使用 `N10P`，这个包和 `slam_config` 依然是最直接的入口
