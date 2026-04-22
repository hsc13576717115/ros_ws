# imu_tf_broadcaster

一个轻量级 Python 工具包，用来把 `sensor_msgs/msg/Imu` 的姿态广播成 TF。它适合快速实验、验证坐标系，或者在没有完整传感器驱动封装时先把 IMU 挂进 TF 树。

## 作用

- 订阅一个 IMU 话题
- 读取四元数姿态
- 广播 `world_frame_id -> imu_frame_id` 的 TF
- 平移量通过参数 `position_x/y/z` 指定

## 主要入口

- 节点：`imu_tf_broadcaster`
- 启动文件：[`launch/imu_tf_broadcaster.launch.py`](launch/imu_tf_broadcaster.launch.py)

## 常用命令

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash

ros2 launch imu_tf_broadcaster imu_tf_broadcaster.launch.py \
  imu_topic:=/imu/data_raw \
  world_frame_id:=base_link \
  imu_frame_id:=gyro_link
```

## 常用参数

- `imu_topic`：默认 `/imu/data_raw`
- `world_frame_id`：默认 `world`
- `imu_frame_id`：默认 `gyro_link`
- `position_x/y/z`：默认 `1, 1, 0`

## 备注

- 当前工作区默认链路里更常用的是 [`slam_config/launch/tf_static_launch.py`](../slam_config/launch/tf_static_launch.py)
- 这个包更适合调试、实验和临时坐标系接入
