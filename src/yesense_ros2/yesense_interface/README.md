# yesense_interface

Yesense 自定义消息接口包。这个包不启动节点，主要给 `yesense_std_ros2`、`vmc_quadruped_controller` 和其他订阅者提供统一的消息定义。

## 提供的能力

- IMU 原始数据消息
- 欧拉角与姿态类消息
- 最小姿态/导航数据消息
- 扩展导航与传感器状态消息

## 常用消息

- `ImuData`
- `EulerOnly`
- `AttitudeMinVru`
- `NavMin`
- `NavAll`
- `RobotLord`

完整定义可在 [`msg/`](msg) 目录查看。

## 在工作区中的角色

- 上游：无，纯接口包
- 下游：
  - [`yesense_std_ros2`](../yesense_std_ros2/README.md)
  - [`vmc_quadruped_controller`](../../VMC_Quadruped_Controller/README.md)

## 常用命令

```bash
source /opt/ros/humble/setup.bash
cd /home/orangepi/ros_ws
colcon build --packages-select yesense_interface yesense_std_ros2
```
