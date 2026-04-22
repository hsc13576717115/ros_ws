# slam_config

兼容的二维 SLAM 配置包，主要服务 `N10P + Yesense + Cartographer` 这一条老链路。当前推荐主链路已经切到 `MID360 + FAST-LIO`，但这个包依然适合现场回退、兼容旧硬件和做基础排障。

## 作用

- 发布固定 TF
- 启动 `lslidar_driver`
- 可选启动 `yesense_std_ros2`
- 启动 `cartographer_node` 和栅格地图节点

## 主要入口

- 主启动文件：[`launch/slam_launch.py`](launch/slam_launch.py)
- 静态 TF：[`launch/tf_static_launch.py`](launch/tf_static_launch.py)
- Cartographer 配置：
  - [`config/n10p_2d_simple.lua`](config/n10p_2d_simple.lua)
  - [`config/n10p_2d.lua`](config/n10p_2d.lua)
- TF 参数：[`config/tf_static.yaml`](config/tf_static.yaml)

## 常用命令

启动兼容二维 SLAM：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash

ros2 launch slam_config slam_launch.py start_imu:=true
```

如果要把兼容 Nav2 一起拉起：

```bash
ros2 launch nav2_config slam_navigation.py start_imu:=false start_yolo:=false
```

## 与其他包的关系

- 上游：
  - [`lslidar_driver`](../lslidar_driver/README.md)
  - [`yesense_std_ros2`](../yesense_ros2/yesense_std_ros2/README.md)
- 下游：
  - [`nav2_config`](../nav2_config/README.md)

## 备注

- 如果你用的还是 `N10P` 而不是 `MID360`，从这个包开始最自然
- 如果你想切回当前推荐主链路，请看 [`nav2_config`](../nav2_config/README.md) 和 [`fast_lio`](../FAST_LIO_ROS2/README.md)
