# ROS2 建图定位导航使用指南

适用工作区：`/home/orangepi/ros_ws`  
适用场景：四足狗手柄控制 + Cartographer 建图 + Nav2 导航（可手柄随时接管）

## 1. 编译与环境

```bash
cd /home/orangepi/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vmc_quadruped_controller slam_config nav2_config yesense_std_ros2 lslidar_driver
source install/setup.bash
```

每个新终端都需要执行：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash
```

## 2. 推荐启动方式（手柄 + 导航不冲突）

### 终端1：只启动狗本体（手柄控制）

```bash
ros2 launch vmc_quadruped_controller dog.launch.py
```

说明：
- 该 launch 启动 `joy_node`、`yesense`、`foots`
- 此时只手柄控制，不跑导航

### 终端2：启动建图 + 定位 + 导航

```bash
ros2 launch nav2_config dog_slam_navigation.py
```

说明：
- 默认 `start_imu:=false`，避免和 `dog.launch.py` 重复启动 IMU
- 导航过程手柄可随时接管，松开后会恢复自动控制
- 已增加自动启动保护：只有检测到 `odom -> base_link` TF 就绪后，Nav2 才会激活

## 3. 如果不运行 dog.launch（单独跑导航）

需要让导航 launch 自己启动 IMU：

```bash
ros2 launch nav2_config dog_slam_navigation.py start_imu:=true
```

## 4. 设置目标点导航

### 方式A：RViz 2D Goal Pose

- 在 RViz 中选择 `2D Goal Pose`
- 在地图中点击目标点并拖动朝向

### 方式B：命令行发送坐标目标

```bash
ros2 run nav2_config send_goal.py --x 1.0 --y 0.5 --yaw 0.0
```

参数说明：
- `--x` / `--y`：目标点坐标（单位米，`map` 坐标系）
- `--yaw`：目标朝向（弧度）

### 方式C：预设多点（约20点）自动巡航

预设点文件：

`/home/orangepi/ros_ws/src/nav2_config/config/preset_waypoints.yaml`

启动命令（开启预设点任务）：

```bash
ros2 launch nav2_config dog_slam_navigation.py \
  start_preset_mission:=true \
  waypoint_file:=/home/orangepi/ros_ws/src/nav2_config/config/preset_waypoints.yaml
```

可选参数：
- `preset_mission_loop:=true`：最后一个点完成后循环执行
- `start_preset_mission:=false`：仅显示预设点，不自动执行任务

说明：
- 文件里已预留 `P01` 到 `P20` 共20个点
- 每个点支持字段：`name`、`x`、`y`、`yaw_deg`、`enabled`
- 可先把不想执行的点设为 `enabled: false`

## 5. RViz 里应看到的内容

使用 `slam_nav.rviz` 时，建议确认以下显示已开启：

- 目标点：`/goal_pose`
- 全局路径：`/plan`
- 局部路径（若有）：`/local_plan`
- 速度箭头：`/cmd_vel_arrow`
- 预设任务轨迹：`/preset_route`
- 预设目标点与序号：`/preset_waypoints`
- 当前正在执行的预设目标：`/preset_current_goal`
- 激光：`/scan`
- 地图：`/map`

## 6. 常用运行状态检查

```bash
ros2 topic echo /imu/data_raw --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /scan
ros2 topic echo /cmd_vel --once
```

关键点：
- `odom -> base_link` 必须存在，否则 Nav2 无法激活
- `imu/data_raw` 必须有数据，否则 Cartographer 可能不出 `odom`

## 7. 常见问题与处理

### 问题1：`Timed out waiting for transform from base_link to odom`

原因：
- IMU 没启动，或 IMU 话题不对

处理：
1. 如果已跑 `dog.launch.py`，确认 `yesense` 正常且 `/imu/data_raw` 有数据  
2. 如果未跑 `dog.launch.py`，使用：
   ```bash
   ros2 launch nav2_config dog_slam_navigation.py start_imu:=true
   ```

### 问题2：RViz 报 `Message Filter dropping message ... queue is full`

原因：
- 通常是 TF 链不完整（尤其缺 `odom`）导致

处理：
- 先解决 `odom -> base_link` 变换，再看 RViz

### 问题3：手柄和导航指令冲突

当前已处理为“手柄优先”：
- 手柄有动作时，立即接管
- 手柄停止后，自动控制继续生效

## 8. 地图保存（可选）

完成建图后可保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

会生成：
- `~/my_map.yaml`
- `~/my_map.pgm`

## 9. 仅定位导航（使用已有地图）

```bash
ros2 launch nav2_config navigation_only.py map:=/home/orangepi/my_map.yaml
```

如果不跑 `dog.launch.py`，追加：

```bash
start_imu:=true
```

## 10. 关闭顺序

建议先关闭导航终端，再关闭 `dog.launch.py` 终端，避免 lifecycle 在退出阶段报连锁错误。
RDP：
1）先找是谁占了 3389

在香橙派执行：

sudo ss -lntp | grep ':3389'

你会看到类似 xrdp 或其他进程名。

2）如果是 xrdp：直接停掉并禁用（推荐）
sudo systemctl disable --now xrdp
sudo systemctl disable --now xrdp-sesman

再确认 3389 释放了：

sudo ss -lntp | grep ':3389' || echo "3389 free"
3）重启 GNOME Remote Desktop（让它重新绑定 3389）
systemctl --user restart gnome-remote-desktop
systemctl --user status gnome-remote-desktop --no-pager

如果你是在 tty/ssh 里跑的，但服务属于图形会话用户，有时需要你在桌面会话里执行一次重启；不过你刚才能看到 active，说明 user service 没问题。

4）Windows 连接

Windows：Win + R → mstsc → 填 香橙派 IP → 用户名（一般 orangepi）+ 密码登录。