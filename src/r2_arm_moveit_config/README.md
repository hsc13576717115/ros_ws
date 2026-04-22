# r2_arm_moveit_config

`r2_arm_moveit_config` 为 `r2_arm_control` 提供 `MoveIt`、`SRDF`、运动学、规划器和 RViz 配置。它更多是“规划与可视化层”，而不是底层驱动层。

## 作用

- 组织 `MoveIt` 配置文件
- 结合 `r2_arm_control/bringup.launch.py` 拉起控制链
- 提供机械臂的 RViz 可视化
- 作为上层调试和规划的入口

## 主要入口

- 启动文件：[`launch/moveit.launch.py`](launch/moveit.launch.py)
- 语义模型：[`srdf/r2_arm.srdf`](srdf/r2_arm.srdf)
- 规划参数：
  - [`config/kinematics.yaml`](config/kinematics.yaml)
  - [`config/ompl_planning.yaml`](config/ompl_planning.yaml)
  - [`config/joint_limits.yaml`](config/joint_limits.yaml)
  - [`config/moveit_controllers.yaml`](config/moveit_controllers.yaml)

## 当前行为说明

- 启动时会包含 `r2_arm_control/bringup.launch.py`
- 当前 `arm.yaml` 中的 `executor_mode` 是 `mit_native_5var`
- 在这个模式下，launch 会优先保留硬件、状态桥和 `MoveToXZ` 服务节点；`move_group` 是否真正参与由 launch 内部逻辑决定

## 常用命令

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/ros_ws/install/setup.bash

ros2 launch r2_arm_moveit_config moveit.launch.py start_rviz:=true
```

如果现场没有显示器或远程图形环境不可用：

```bash
ros2 launch r2_arm_moveit_config moveit.launch.py start_rviz:=false
```

## 备注

- 如果你只想让机械臂跑起来，不一定要先从这个包进，`r2_arm_control/system.launch.py` 更偏“整机可用”
- 如果你在调机械臂模型、关节限制、规划器和 RViz，这个包才是主入口
