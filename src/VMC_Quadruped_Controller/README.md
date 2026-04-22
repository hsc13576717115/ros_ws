# vmc_quadruped_controller

四足底盘控制与整机启动入口包。它负责把手柄控制、导航输出、IMU 姿态和机械臂状态整合起来，并通过 Unitree SDK 驱动底盘动作。

## 作用

- 运行四足底盘主控制节点 `foots`
- 处理 `/joy` 手柄输入
- 接收导航桥接出来的 `/move_cmd`
- 与 `r2_arm_control` 的状态和目标接口联动
- 提供“手柄优先”的人工接管逻辑

## 主要入口

- 手动/基础启动：[`launch/dog.launch.py`](launch/dog.launch.py)
- 当前全栈入口：[`launch/r2_full.launch.py`](launch/r2_full.launch.py)
- 简化导航测试：[`launch/nav_liner.launch.py`](launch/nav_liner.launch.py)
- 可执行程序：
  - `foots`
  - `motor`
  - `navigator`
  - `nav_liner`

## 关键接口

- 订阅：
  - `/joy`
  - `/move_cmd`
  - `euler_only`
  - `/r2/arm/state`
  - `/r2/arm/state_machine_state`
- 与导航层的核心契约消息：
  - [`msg/MoveCmd.msg`](msg/MoveCmd.msg)
- 与机械臂联动：
  - 发布 `/r2/arm/target_xz`

## 常用命令

只启动底盘、手柄与可选机械臂系统：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash

ros2 launch vmc_quadruped_controller dog.launch.py
```

启动当前推荐的整机全栈：

```bash
ros2 launch vmc_quadruped_controller r2_full.launch.py start_yolo:=false
```

## 与其他包的关系

- 上游导航：[`nav2_config`](../nav2_config/README.md)
- 上游 IMU：[`yesense_std_ros2`](../yesense_ros2/yesense_std_ros2/README.md)
- 上游机械臂：[`r2_arm_control`](../r2_arm_control/README.md)

## 备注

- `foots` 是整机最关键的落地节点之一，因为导航命令最终都要经过它
- 当前代码里已经考虑了手柄动作覆盖自动导航的情况，适合现场调试和安全接管
