# nav2_config

工作区里的导航集成包。它不实现底盘本体控制，而是把 `FAST-LIO`、`Cartographer`、`Nav2`、目标发送、点云投影、`cmd_vel -> MoveCmd` 桥接、预设点任务这些能力组织在一起。

## 作用

- 管理 Nav2 参数、行为树和 RViz 配置
- 提供当前主链路 `MID360 + FAST-LIO + Nav2`
- 保留兼容链路 `N10P + Cartographer + Nav2`
- 把 Nav2 的 `/cmd_vel` 映射成四足底盘消费的 `/move_cmd`
- 提供目标发送、预设点巡航和 Nav2 延迟激活工具

## 主要入口

- 当前主链路：
  - [`launch/mid360_navigation.py`](launch/mid360_navigation.py)
  - [`launch/dog_slam_navigation.py`](launch/dog_slam_navigation.py)
- 兼容链路：
  - [`launch/slam_navigation.py`](launch/slam_navigation.py)
  - [`launch/navigation_only.py`](launch/navigation_only.py)
- 常用脚本：
  - [`scripts/send_goal.py`](scripts/send_goal.py)
  - [`scripts/cmd_vel_to_move_cmd.py`](scripts/cmd_vel_to_move_cmd.py)
  - [`scripts/pointcloud_to_scan.py`](scripts/pointcloud_to_scan.py)
  - [`scripts/wait_for_odom_activate_nav2.py`](scripts/wait_for_odom_activate_nav2.py)
  - [`scripts/preset_waypoint_mission.py`](scripts/preset_waypoint_mission.py)

## 常用命令

当前主链路：

```bash
ros2 launch nav2_config dog_slam_navigation.py start_yolo:=false
```

只起导航核心：

```bash
ros2 launch nav2_config mid360_navigation.py start_yolo:=false
```

已有地图做纯定位导航：

```bash
ros2 launch nav2_config navigation_only.py map:=/home/orangepi/my_map.yaml
```

发送单个目标：

```bash
ros2 run nav2_config send_goal.py --x 1.0 --y 0.0 --yaw 0.0
```

## 关键配置

- 当前主链路 Nav2 参数：[`config/nav2_fastlio_params.yaml`](config/nav2_fastlio_params.yaml)
- 兼容链路 Nav2 参数：[`config/nav2_slam_params.yaml`](config/nav2_slam_params.yaml)
- 纯定位参数：[`config/nav2_params.yaml`](config/nav2_params.yaml)
- 点云转 LaserScan：[`config/pointcloud_to_scan.yaml`](config/pointcloud_to_scan.yaml)
- 预设点：[`config/preset_waypoints.yaml`](config/preset_waypoints.yaml)

## 与其他包的关系

- 上游：
  - [`fast_lio`](../FAST_LIO_ROS2/README.md)
  - [`slam_config`](../slam_config/README.md)
  - [`yolov11n_rknn`](../yolov11n_rknn/README.md)
- 下游：
  - [`vmc_quadruped_controller`](../VMC_Quadruped_Controller/README.md)

## 备注

- 这个包是整机导航集成中心，很多“机器人能不能自己走”问题最后都要回到这里排查
- `dog_slam_navigation.py` 现在已经切到 `MID360 + FAST-LIO`，不再是旧 README 里那条默认的 `N10P + Cartographer` 主路径
