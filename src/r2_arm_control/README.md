# r2_arm_control

自研 `2R` 机械臂控制包，负责硬件接入、IK、`MoveToXZ` 服务、状态发布、GPIO/手柄状态机，以及与四足底盘之间的联动。

## 作用

- 管理 Damiao 电机和 `ros2_control` 硬件层
- 提供 `x/z` 末端目标到关节目标的 IK 与执行
- 发布机械臂运动状态和末端点位
- 提供 D-Pad 触发的状态机与 GPIO 联动逻辑

## 主要入口

- 推荐系统入口：[`launch/system.launch.py`](launch/system.launch.py)
- 底层控制入口：[`launch/bringup.launch.py`](launch/bringup.launch.py)
- 主要可执行程序：
  - `arm_command_server_node`
  - `ik_commander_node`
  - `ee_state_publisher_node`
  - `joint_state_timestamp_bridge_node`
  - `arm_state_machine_node.py`
  - `dpad_gpio_toggle_node.py`

## 关键接口

- 服务：`/r2/arm/move_to_xz`
- 状态话题：`/r2/arm/state`
- 末端点：`/r2/arm/end_effector_point`
- 反馈健康状态：`/r2/arm/feedback_status`
- 消息：
  - [`msg/ArmCartesianTarget.msg`](msg/ArmCartesianTarget.msg)
  - [`msg/ArmMotionState.msg`](msg/ArmMotionState.msg)
  - [`msg/MitJointCommand.msg`](msg/MitJointCommand.msg)
- 服务定义：[`srv/MoveToXZ.srv`](srv/MoveToXZ.srv)

## 关键配置

- 机械臂几何、增益和执行模式：[`config/arm.yaml`](config/arm.yaml)
- 电机与 CAN 映射：[`config/motors.yaml`](config/motors.yaml)
- ros2_control 控制器：[`config/controllers.yaml`](config/controllers.yaml)
- 机器人模型：[`urdf/r2_arm.urdf.xacro`](urdf/r2_arm.urdf.xacro)

当前配置中，`executor_mode` 是 `mit_native_5var`。

## 常用命令

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash

ros2 launch r2_arm_control system.launch.py
```

调用末端移动服务：

```bash
ros2 service call /r2/arm/move_to_xz r2_arm_control/srv/MoveToXZ "{x: 0.20, z: 0.20}"
```

## 与其他包的关系

- 上层规划与可视化：[`r2_arm_moveit_config`](../r2_arm_moveit_config/README.md)
- 整机联动入口：[`vmc_quadruped_controller`](../VMC_Quadruped_Controller/README.md)

## 备注

- 这是当前工作区里自研程度最高的包之一
- 如果机械臂“能上电但不按预期动作”，优先看 `arm.yaml`、`motors.yaml` 和 `arm_command_server_node.cpp`
