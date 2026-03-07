#!/bin/bash
# Nav2 导航诊断脚本

echo "=========================================="
echo "  Nav2 导航诊断"
echo "=========================================="
echo ""

WS_ROOT="${WS_ROOT:-/home/orangepi/ros_ws}"
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    source "${WS_ROOT}/install/setup.bash"
else
    echo "未找到工作空间环境: ${WS_ROOT}/install/setup.bash"
    exit 1
fi

echo "1. 检查 Cartographer 地图发布"
echo "----------------------------------------"
ros2 topic info /map
echo ""

echo "2. 检查代价地图发布"
echo "----------------------------------------"
echo "Global Costmap:"
ros2 topic info /global_costmap/costmap
echo ""
echo "Local Costmap:"
ros2 topic info /local_costmap/costmap
echo ""

echo "3. 检查 Nav2 节点状态"
echo "----------------------------------------"
echo "Nodes:"
ros2 node list | grep -E "controller|planner|bt_navigator|lifecycle"
echo ""

echo "Lifecycle 状态:"
echo "  Controller: $(ros2 lifecycle get /controller_server 2>/dev/null | head -1)"
echo "  Planner:    $(ros2 lifecycle get /planner_server 2>/dev/null | head -1)"
echo "  BT Navigator: $(ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)"
echo ""

echo "4. 检查路径话题发布者"
echo "----------------------------------------"
echo "Global Plan (/global_plan):"
ros2 topic info /global_plan
echo ""
echo "Local Plan (/local_plan):"
ros2 topic info /local_plan
echo ""
echo "Plan (/plan):"
ros2 topic info /plan
echo ""

echo "5. 检查 cmd_vel 话题"
echo "----------------------------------------"
ros2 topic info /cmd_vel
echo ""

echo "=========================================="
echo "  测试导航目标"
echo "=========================================="
echo ""
echo "在 RViz 中:"
echo "  1. 等待地图加载完成"
echo "  2. 使用 '2D Pose Estimate' 设置机器人初始位置"
echo "  3. 使用 'Nav2 Goal' 点击目标位置"
echo ""
echo "或者用命令行测试:"
echo "  ros2 run nav2_config send_goal.py --x 1.0 --y 0.0 --yaw 0.0"
echo ""
