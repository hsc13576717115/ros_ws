# quadruped_navigation
独立导航包 - 专为四足机器人设计

## 功能
- 实时SLAM建图（Cartographer）
- 实时导航到目标点
- 仅使用2D雷达 + IMU（无需额外里程计）
- 完全独立，不修改现有代码

## 依赖
- ROS 2 Humble
- cartographer
- cartographer_ros
- navigation2
- 现有 vmc_quadruped_controller

## 使用

### 建图+导航（一体化）
```bash
# 终端1：启动机器人
ros2 launch vmc_quadruped_controller dog.launch.py

# 终端2：启动导航系统（实时建图+导航）
ros2 launch quadruped_navigation realtime_nav.launch.py

# 终端3：发送导航目标
ros2 action send_goal /goal_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 2.0, y: 1.0}}}}"
```

### 仅保存地图
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map.yaml
```

## 参数说明
关键参数在 `config/` 目录下：
- cartographer_config.lua - SLAM配置
- nav2_params.yaml - Navigation2配置
- dwb_params.yaml - DWB控制器配置

## 适配硬件
修改以下参数以适应您的硬件：
1. 雷达型号：config/cartographer_config.lua
2. IMU型号：src/imu_converter.cpp
3. 机器人尺寸：config/nav2_params.yaml (footprint)
