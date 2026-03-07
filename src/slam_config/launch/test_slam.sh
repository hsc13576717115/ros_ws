#!/bin/bash
# SLAM 功能测试脚本
# 用于验证 Cartographer 2D SLAM 配置是否正确

set -e

echo "=========================================="
echo "  Cartographer 2D SLAM 功能测试"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# ============================================================================
# 1. 环境检查
# ============================================================================
echo "1. 检查 ROS2 环境..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    check_pass "ROS2 Humble 环境已加载"
else
    check_fail "ROS2 Humble 未找到"
    exit 1
fi

if [ -f /home/orangepi/ros_ws/install/setup.bash ]; then
    source /home/orangepi/ros_ws/install/setup.bash
    check_pass "工作空间环境已加载"
else
    check_fail "工作空间未构建"
    exit 1
fi

echo ""
echo "2. 检查包安装..."
# 检查 Cartographer
if ros2 pkg list | grep -q "cartographer_ros"; then
    check_pass "cartographer_ros 已安装"
else
    check_fail "cartographer_ros 未安装"
    exit 1
fi

# 检查传感器驱动
if ros2 pkg list | grep -q "lslidar_driver"; then
    check_pass "lslidar_driver 已安装"
else
    check_fail "lslidar_driver 未安装"
    exit 1
fi

if ros2 pkg list | grep -q "yesense_std_ros2"; then
    check_pass "yesense_std_ros2 已安装"
else
    check_fail "yesense_std_ros2 未安装"
    exit 1
fi

# 检查 SLAM 配置包
if ros2 pkg list | grep -q "slam_config"; then
    check_pass "slam_config 已安装"
else
    check_fail "slam_config 未安装"
    exit 1
fi

echo ""
echo "3. 检查可执行文件..."
if [ -f /opt/ros/humble/lib/cartographer_ros/cartographer_node ]; then
    check_pass "cartographer_node 可执行文件存在"
else
    check_fail "cartographer_node 可执行文件不存在"
    exit 1
fi

if [ -f /opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node ]; then
    check_pass "cartographer_occupancy_grid_node 可执行文件存在"
else
    check_fail "cartographer_occupancy_grid_node 可执行文件不存在"
    exit 1
fi

echo ""
echo "4. 检查配置文件..."
CONFIG_DIR="/home/orangepi/ros_ws/src/slam_config/config"
LAUNCH_DIR="/home/orangepi/ros_ws/src/slam_config/launch"

if [ -f "$CONFIG_DIR/n10p_2d.lua" ]; then
    check_pass "Cartographer Lua 配置文件存在"
else
    check_fail "Cartographer Lua 配置文件不存在"
    exit 1
fi

if [ -f "$CONFIG_DIR/tf_static.yaml" ]; then
    check_pass "TF 静态配置文件存在"
else
    check_fail "TF 静态配置文件不存在"
    exit 1
fi

if [ -f "$LAUNCH_DIR/slam_launch.py" ]; then
    check_pass "SLAM 启动文件存在"
else
    check_fail "SLAM 启动文件不存在"
    exit 1
fi

echo ""
echo "=========================================="
echo "  所有检查通过!"
echo "=========================================="
echo ""
echo "5. 测试步骤:"
echo ""
echo "   步骤 1: 启动 SLAM"
echo "   ----------------------------------------"
echo "   ros2 launch slam_config slam_launch.py"
echo ""
echo "   步骤 2: 在另一个终端检查 TF 树"
echo "   ----------------------------------------"
echo "   ros2 run tf2_ros view_frames"
echo "   # 预期看到: map -> odom -> base_link -> laser/gyro_link"
echo ""
echo "   步骤 3: 检查话题"
echo "   ----------------------------------------"
echo "   ros2 topic list"
echo "   # 应该看到: /scan, /imu/data_raw, /map, /map_updates"
echo ""
echo "   步骤 4: 查看传感器数据"
echo "   ----------------------------------------"
echo "   ros2 topic echo /scan --once"
echo "   ros2 topic echo /imu/data_raw --once"
echo ""
echo "   步骤 5: 查看地图频率"
echo "   ----------------------------------------"
echo "   ros2 topic hz /map"
echo ""
echo "   步骤 6: 在 RViz 中观察建图效果"
echo "   ----------------------------------------"
echo "   # RViz 应该自动启动，显示地图和激光扫描"
echo ""
echo "   步骤 7: 保存地图"
echo "   ----------------------------------------"
echo "   ros2 run nav2_map_server map_saver_cli -f ~/map"
echo ""
echo "=========================================="
