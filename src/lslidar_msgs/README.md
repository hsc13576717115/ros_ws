# lslidar_msgs

Leishen 激光雷达消息接口包。这个包本身不启动节点，主要为 `lslidar_driver` 提供消息定义。

## 提供的消息

- [`msg/LslidarPacket.msg`](msg/LslidarPacket.msg)
- [`msg/LslidarScan.msg`](msg/LslidarScan.msg)
- [`msg/LslidarSweep.msg`](msg/LslidarSweep.msg)
- [`msg/LslidarPoint.msg`](msg/LslidarPoint.msg)
- [`msg/LslidarDifop.msg`](msg/LslidarDifop.msg)

## 在工作区中的角色

- 上游：无，纯接口包
- 下游：[`lslidar_driver`](../lslidar_driver/README.md)
- 适用链路：`N10P + Cartographer` 的兼容方案

## 常用命令

```bash
source /opt/ros/humble/setup.bash
cd /home/orangepi/ros_ws
colcon build --packages-select lslidar_msgs lslidar_driver
```

## 备注

- 如果只跑 `MID360 + FAST-LIO` 主链路，这个包通常不会直接参与运行
- 只要编译成功，运行时一般由 `lslidar_driver` 自动使用
