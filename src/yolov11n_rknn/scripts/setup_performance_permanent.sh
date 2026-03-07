#!/bin/bash
# 设置永久性能模式 - 创建 systemd 服务（自动检测 NPU）
set -euo pipefail

echo "=========================================="
echo "设置永久性能模式（自动检测平台）"
echo "=========================================="
echo ""

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then
    echo "❌ 此脚本需要 sudo 权限运行"
    echo "   请使用: sudo bash scripts/setup_performance_permanent.sh"
    exit 1
fi

# 检测 NPU 设备
NPU_DEVICE=""
if [ -d /sys/class/devfreq ]; then
    NPU_DEVICE=$(find /sys/class/devfreq -name "*npu*" -o -name "*NPU*" 2>/dev/null | head -1)
fi

if [ -z "$NPU_DEVICE" ]; then
    echo "⚠️  警告: 未检测到 NPU 设备"
    echo "   将仅设置 CPU 性能模式"
    NPU_ENABLED="false"
else
    NPU_BASENAME=$(basename "$NPU_DEVICE")
    echo "✅ 检测到 NPU 设备: $NPU_BASENAME"
    NPU_ENABLED="true"
fi

# 检测平台信息
if [ -f /proc/device-tree/model ]; then
    PLATFORM=$(tr -d '\0' < /proc/device-tree/model)
    echo "📱 平台: $PLATFORM"
fi

echo ""

# 创建 systemd 服务文件
SERVICE_NAME="rk3588-performance.service"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"

cat > $SERVICE_FILE << EOF
[Unit]
Description=Set RKNN CPU/NPU to Performance Mode
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > \$gov; done'
EOF

# 如果检测到 NPU，添加 NPU 相关配置
if [ "$NPU_ENABLED" = "true" ]; then
    cat >> $SERVICE_FILE << EOF
ExecStart=/bin/bash -c 'echo performance > /sys/class/devfreq/${NPU_BASENAME}/governor 2>/dev/null || true'
ExecStart=/bin/bash -c 'echo 1000000000 > /sys/class/devfreq/${NPU_BASENAME}/max_freq 2>/dev/null || true'
EOF
fi

cat >> $SERVICE_FILE << 'EOF'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

echo "✅ 创建服务文件: ${SERVICE_FILE}"

# 重载 systemd 配置
systemctl daemon-reload
echo "✅ 重载 systemd 配置"

# 启用服务（开机自启）
systemctl enable "${SERVICE_NAME}"
echo "✅ 启用开机自启"

# 立即启动服务
systemctl start "${SERVICE_NAME}"
echo "✅ 立即启动服务"

# 验证服务状态
sleep 1
systemctl status "${SERVICE_NAME}" --no-pager
echo ""

echo "=========================================="
echo "✅ 永久性能模式设置完成！"
echo "=========================================="
echo ""
echo "服务已设置为开机自启，重启后自动生效。"
echo ""
echo "管理命令："
echo "  查看状态: systemctl status rk3588-performance.service"
echo "  手动启动: systemctl start rk3588-performance.service"
echo "  禁用自启: systemctl disable rk3588-performance.service"
echo ""
echo "验证当前模式："
echo "  cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
echo "  cat /sys/class/devfreq/fdab0000.npu/governor"
echo "=========================================="
