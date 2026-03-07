#!/bin/bash
# 激活 Nav2 节点脚本

echo "=========================================="
echo "  激活 Nav2 导航节点"
echo "=========================================="
echo ""

WS_ROOT="${WS_ROOT:-/home/orangepi/ros_ws}"
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    source "${WS_ROOT}/install/setup.bash"
else
    echo "未找到工作空间环境: ${WS_ROOT}/install/setup.bash"
    exit 1
fi

echo "1. 激活 controller_server..."
ros2 lifecycle set /controller_server activate

echo ""
echo "2. 配置 planner_server..."
ros2 lifecycle set /planner_server configure

echo ""
echo "3. 激活 planner_server..."
ros2 lifecycle set /planner_server activate

echo ""
echo "4. 配置 bt_navigator..."
ros2 lifecycle set /bt_navigator configure

echo ""
echo "5. 激活 bt_navigator..."
ros2 lifecycle set /bt_navigator activate

echo ""
echo "6. 配置 smoother_server..."
ros2 lifecycle set /smoother_server configure

echo ""
echo "7. 激活 smoother_server..."
ros2 lifecycle set /smoother_server activate

echo ""
echo "8. 配置 behavior_server..."
ros2 lifecycle set /behavior_server configure

echo ""
echo "9. 激活 behavior_server..."
ros2 lifecycle set /behavior_server activate

echo ""
echo "=========================================="
echo "  Nav2 节点激活完成！"
echo "=========================================="
echo ""
echo "现在可以在 RViz 中使用 'Nav2 Goal' 工具设置导航目标"
echo ""
echo "检查节点状态:"
echo "  ros2 lifecycle get /controller_server"
echo "  ros2 lifecycle get /planner_server"
echo "  ros2 lifecycle get /bt_navigator"
