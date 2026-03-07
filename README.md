# ros_ws

面向 `ROS 2 Humble` 的四足机器人工作区，当前主要用于：

- 手柄控制四足底层
- `2D LiDAR + IMU` 感知接入
- `Cartographer` 实时建图与定位
- `Nav2` 室内低速导航与精确停车
- 可选的 `RKNN YOLO` 目标检测

这个仓库跟踪的是实际工作区 `/home/orangepi/ros_ws`，后续直接在工作区根目录执行 `git add / commit / push` 即可。

## 工作区结构

- `src/VMC_Quadruped_Controller`
  四足底层控制、手柄接管、`MoveCmd` 消息定义、`dog.launch.py`
- `src/nav2_config`
  Nav2 参数、行为树、launch、发目标脚本、`cmd_vel -> MoveCmd` 桥接
- `src/slam_config`
  Cartographer 配置、静态 TF、SLAM launch
- `src/lslidar_driver`
  雷达驱动
- `src/lslidar_msgs`
  雷达消息定义
- `src/yesense_ros2`
  Yesense IMU 驱动与消息
- `src/serial_ros2`
  串口基础库
- `src/imu_tf_broadcaster`
  IMU TF 广播辅助节点
- `src/yolov11n_rknn`
  RKNN 推理与检测节点，可选
- [NAVIGATION_USAGE_GUIDE.md](./NAVIGATION_USAGE_GUIDE.md)
  更完整的建图、定位、导航使用说明
- [GIT_WORKFLOW.md](./GIT_WORKFLOW.md)
  工作区 Git 提交、分支、合并、回滚与回溯说明

## 当前导航链路

当前实际在用的 SLAM 导航链路是：

- `Cartographer + Nav2`
- `SmacPlannerHybrid`
- `RotationShimController + RegulatedPurePursuit`
- `VelocitySmoother`
- `cmd_vel` 通过桥接转换成四足底层 `MoveCmd.step_x / step_y`

关键配置入口：

- [src/nav2_config/config/nav2_slam_params.yaml](./src/nav2_config/config/nav2_slam_params.yaml)
- [src/nav2_config/launch/slam_navigation.py](./src/nav2_config/launch/slam_navigation.py)
- [src/nav2_config/behavior_trees/navigate_to_pose_w_distance_replanning_and_recovery.xml](./src/nav2_config/behavior_trees/navigate_to_pose_w_distance_replanning_and_recovery.xml)
- [src/nav2_config/scripts/cmd_vel_to_move_cmd.py](./src/nav2_config/scripts/cmd_vel_to_move_cmd.py)

## 快速开始

### 1. 构建

```bash
cd /home/orangepi/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  vmc_quadruped_controller \
  slam_config \
  nav2_config \
  yesense_std_ros2 \
  lslidar_driver
source install/setup.bash
```

如果需要整工作区编译：

```bash
cd /home/orangepi/ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

每个新终端都需要：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash
```

### 2. 典型启动方式

终端 A，启动四足本体和手柄控制：

```bash
ros2 launch vmc_quadruped_controller dog.launch.py
```

终端 B，启动 SLAM + 定位 + 导航：

```bash
ros2 launch nav2_config dog_slam_navigation.py
```

这条链路的特点：

- 手柄优先，手动动作时可立即接管
- 导航恢复后继续使用自动控制
- Nav2 会等待 `odom -> base_link` TF 就绪后再激活

### 3. 发送导航目标

RViz 中使用 `2D Goal Pose`，或者命令行：

```bash
ros2 run nav2_config send_goal.py --x 1.0 --y 0.5 --yaw 0.0
```

### 4. 使用已有地图做纯定位导航

```bash
ros2 launch nav2_config navigation_only.py map:=/home/orangepi/my_map.yaml
```

## 仓库维护说明

- 顶层 `.gitignore` 已忽略 `build/ install/ log/` 等工作区产物
- 当前仓库以工作区根目录为 Git 根目录
- 原先 `src/VMC_Quadruped_Controller` 与 `src/serial_ros2` 的嵌套 Git 元数据已备份到 `.embedded_git_backups/`
- 以后建议始终在 `/home/orangepi/ros_ws` 根目录下执行 Git 命令

常用命令：

```bash
cd /home/orangepi/ros_ws
git status
git add .
git commit -m "update workspace"
git push
```

## 备注

- `src/yolov11n_rknn` 包含模型与数据文件，仓库体积会比较大
- `src/yesense_ros2` 是直接纳入工作区的源码目录，不是子模块
- 更详细的导航运行步骤、排障和预设点说明见 [NAVIGATION_USAGE_GUIDE.md](./NAVIGATION_USAGE_GUIDE.md)
- Git 日常使用和回溯流程见 [GIT_WORKFLOW.md](./GIT_WORKFLOW.md)
