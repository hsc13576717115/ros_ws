#!/bin/bash
# 硬件连接测试脚本
# 在启动 SLAM 之前验证传感器硬件是否正常连接

echo "=========================================="
echo "  传感器硬件测试"
echo "=========================================="
echo ""

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# 检查串口设备
echo "1. 检查串口设备..."
echo ""

# 检查激光雷达串口 (根据配置文件是 /dev/ttyS4)
if [ -e /dev/ttyS4 ]; then
    check_pass "激光雷达串口 /dev/ttyS4 存在"
    ls -l /dev/ttyS4
else
    check_fail "激光雷达串口 /dev/ttyS4 不存在"
    echo "  可用串口设备:"
    ls -l /dev/tty* /dev/ttyS* 2>/dev/null | grep -v "cannot access"
fi
echo ""

# 检查 IMU 串口
if [ -e /dev/ttyUSB0 ]; then
    check_pass "IMU 串口 /dev/ttyUSB0 存在"
    ls -l /dev/ttyUSB0
else
    check_fail "IMU 串口 /dev/ttyUSB0 不存在"
    echo "  可用 USB 设备:"
    ls -l /dev/ttyUSB* 2>/dev/null || echo "  无 USB 串口设备"
fi
echo ""

# 检查串口权限
echo "2. 检查串口权限..."
if groups | grep -q dialout; then
    check_pass "用户在 dialout 组中"
else
    check_warn "用户不在 dialout 组，可能需要手动设置权限"
    echo "  运行: sudo usermod -a -G dialout \$USER"
    echo "  或: sudo chmod 666 /dev/ttyS4 /dev/ttyUSB0"
fi
echo ""

# 检查网络配置 (如果使用网口雷达)
echo "3. 检查网络配置..."
if ip addr show | grep -q "192.168.1"; then
    check_pass "找到 192.168.1.x 网络配置"
    ip addr show | grep "inet 192.168.1"
else
    check_warn "未找到 192.168.1.x 网络配置"
    echo "  网口雷达需要配置静态 IP: 192.168.1.102"
fi
echo ""

# 提示用户如何测试传感器
echo "=========================================="
echo "4. 传感器单独测试"
echo "=========================================="
echo ""
echo "测试激光雷达:"
echo "  ros2 launch lslidar_driver lsn10p_launch.py"
echo "  ros2 topic echo /scan"
echo ""
echo "测试 IMU:"
echo "  ros2 launch yesense_std_ros2 yesense_node.launch.py"
echo "  ros2 topic echo /imu/data_raw"
echo ""
echo "=========================================="
