# serial

目录名是 `serial_ros2`，包名是 `serial`。它是一个基础串口通信库，不直接面向机器人业务逻辑，但 `yesense_std_ros2` 会依赖它来访问串口设备。

## 作用

- 提供跨平台风格的串口读写接口
- 作为 `yesense_std_ros2` 等驱动包的底层依赖
- 在当前工作区里不单独启动节点

## 当前工作区中的角色

- 上游：无，基础库
- 下游：[`yesense_std_ros2`](../yesense_ros2/yesense_std_ros2/README.md)

## 常用命令

```bash
source /opt/ros/humble/setup.bash
cd /home/orangepi/ros_ws
colcon build --packages-select serial
```

## 备注

- 如果 Yesense 驱动编译或链接失败，通常需要顺带检查这个包
- 运行整机系统时，一般不需要直接关注它
