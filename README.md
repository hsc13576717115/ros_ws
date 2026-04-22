# ros_ws

面向 `OrangePi + ROS 2 Humble` 的机器人工作空间，集成了四足底盘、`2R` 机械臂、二维/三维激光、IMU、Nav2 导航，以及可选的 `RKNN YOLO` 视觉感知。

这个工作空间同时保留了两条导航链路：

- 当前推荐链路：`Livox MID360 -> livox_ros_driver2 -> fast_lio -> nav2_config/pointcloud_to_scan.py -> Nav2 -> cmd_vel_to_move_cmd.py -> vmc_quadruped_controller`
- 兼容链路：`Leishen N10P -> lslidar_driver + yesense_std_ros2 -> slam_config (Cartographer) -> Nav2`

## 架构概览

```text
/joy
  -> vmc_quadruped_controller/foots
  -> r2_arm_control/arm_state_machine_node

/livox/lidar + /livox/imu
  -> fast_lio
  -> /cloud_registered_body
  -> nav2_config/pointcloud_to_scan.py
  -> /scan
  -> Nav2
  -> /cmd_vel
  -> nav2_config/cmd_vel_to_move_cmd.py
  -> /move_cmd
  -> vmc_quadruped_controller/foots

/scan + /imu/data_raw
  -> slam_config (Cartographer)
  -> /map + odom/base_link
  -> Nav2

camera
  -> yolov11n_rknn
  -> /yolo/detections

r2_arm_control + r2_arm_moveit_config
  -> MoveToXZ 服务 / 末端状态 / ros2_control / MoveIt 配置
```

## 包索引

### 运动控制

- [vmc_quadruped_controller](src/VMC_Quadruped_Controller/README.md)：四足底盘控制、手柄接管、导航指令接入、全栈启动入口
- [r2_arm_control](src/r2_arm_control/README.md)：`2R` 机械臂硬件接入、IK、状态机、`MoveToXZ` 服务
- [r2_arm_moveit_config](src/r2_arm_moveit_config/README.md)：机械臂 `MoveIt`/`ros2_control` 配置与可视化

### 导航与定位

- [nav2_config](src/nav2_config/README.md)：Nav2 参数、行为树、桥接脚本、目标发送工具
- [slam_config](src/slam_config/README.md)：`N10P + IMU + Cartographer` 的兼容 SLAM 配置
- [fast_lio](src/FAST_LIO_ROS2/README.md)：`MID360 + IMU` 紧耦合里程计与点云配准

### 传感器与接口

- [livox_ros_driver2](src/livox_ros_driver2/README.md)：Livox `MID360/HAP` 驱动
- [lslidar_driver](src/lslidar_driver/README.md)：Leishen `M10/N10/N10P` 驱动
- [lslidar_msgs](src/lslidar_msgs/README.md)：Leishen 雷达消息接口
- [yesense_ros2](src/yesense_ros2/readme.md)：Yesense 驱动栈总览
- [yesense_interface](src/yesense_ros2/yesense_interface/README.md)：Yesense 自定义消息接口
- [yesense_std_ros2](src/yesense_ros2/yesense_std_ros2/README.md)：Yesense IMU/GNSS 驱动
- [imu_tf_broadcaster](src/imu_tf_broadcaster/README.md)：把 IMU 姿态广播成 TF 的小工具包

### 点云与支持库

- [perception_pcl stack](src/perception_pcl/README.md)：点云相关依赖栈总览
- [perception_pcl](src/perception_pcl/perception_pcl/README.md)：元包，聚合 `pcl_ros/pcl_conversions`
- [pcl_ros](src/perception_pcl/pcl_ros/README.md)：ROS 与 PCL 组件、TF 转换与点云工具
- [pcl_conversions](src/perception_pcl/pcl_conversions/README.md)：PCL 与 ROS 消息转换库
- [serial](src/serial_ros2/README.md)：串口库，Yesense 驱动依赖
- [yolov11n_rknn](src/yolov11n_rknn/README.md)：可选的 RKNN 目标检测节点

## 使用方法

### 1. 构建工作空间

```bash
source /opt/ros/humble/setup.bash
cd /home/orangepi/ros_ws
colcon build --symlink-install
source install/setup.bash
```

每个新终端都建议执行：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash
```

### 2. 一键启动当前主链路

推荐直接从整机入口启动：

```bash
ros2 launch vmc_quadruped_controller r2_full.launch.py start_yolo:=false
```

说明：

- `r2_full.launch.py` 会先启动 `dog.launch.py`
- 然后再拉起 `nav2_config/dog_slam_navigation.py`
- 如果相机或 NPU 还没准备好，先把 `start_yolo:=false`

### 3. 分步启动当前主链路

终端 A，启动底盘、手柄、机械臂与可选 USB IMU：

```bash
ros2 launch vmc_quadruped_controller dog.launch.py
```

终端 B，启动 `MID360 + FAST-LIO + Nav2`：

```bash
ros2 launch nav2_config dog_slam_navigation.py start_yolo:=false
```

只启动导航核心而不经过四足包装入口时，可用：

```bash
ros2 launch nav2_config mid360_navigation.py start_yolo:=false
```

### 4. 启动兼容的二维 SLAM 链路

如果现场使用的是 `N10P + Yesense`：

```bash
ros2 launch slam_config slam_launch.py start_imu:=true
```

如果要在 SLAM 同时运行时把 Nav2 一起拉起：

```bash
ros2 launch nav2_config slam_navigation.py start_imu:=false start_yolo:=false
```

### 5. 机械臂单独启动

```bash
ros2 launch r2_arm_control system.launch.py
```

如果需要看 MoveIt/RViz 配置：

```bash
ros2 launch r2_arm_moveit_config moveit.launch.py start_rviz:=true
```

### 6. 发送导航目标

```bash
ros2 run nav2_config send_goal.py --x 1.0 --y 0.0 --yaw 0.0
```

如果用已有地图做纯定位导航：

```bash
ros2 launch nav2_config navigation_only.py map:=/home/orangepi/my_map.yaml
```

## 常用排查命令

```bash
ros2 topic echo /move_cmd --once
ros2 topic echo /r2/arm/state --once
ros2 topic echo /imu/data_raw --once
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link
```

主链路里最关键的几个条件是：

- `odom -> base_link` 必须存在
- `Nav2` 必须能读到 `/scan`
- 四足底盘必须能消费 `/move_cmd`
- 如果启用机械臂，`/r2/arm/state` 和 `/r2/arm/feedback_status` 要正常刷新

## 维护建议

- `src/yesense_ros2` 与 `src/perception_pcl` 是多包源码栈，不是单一 ROS 包
- `src/FAST_LIO_ROS2`、`src/livox_ros_driver2`、`src/perception_pcl` 中有第三方代码；这里的 README 以“本工作区如何集成和使用”为主
- 如果后续要把当前本地改动发布到远端仓库，可以直接在 `/home/orangepi/ros_ws` 根目录继续做 `git commit` 和 PR 流程
