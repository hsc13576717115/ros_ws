#!/bin/bash
# YOLOv11n RKNN 性能优化脚本

echo "=========================================="
echo "YOLOv11n RKNN 性能优化"
echo "=========================================="
echo ""

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then
    echo "❌ 此脚本需要 sudo 权限运行"
    echo "   请使用: sudo bash scripts/optimize_performance.sh"
    exit 1
fi

echo "1️⃣ 设置 CPU 为 performance 模式"
echo "----------------------------------------"
for cpu in /sys/devices/system/cpu/cpu[0-7]/cpufreq/scaling_governor; do
    echo performance > $cpu
    echo "✅ CPU $(basename $(dirname $cpu)): performance"
done
echo ""

echo "2️⃣ 设置 NPU 为 performance 模式"
echo "----------------------------------------"
if [ -w /sys/class/devfreq/fdab0000.npu/governor ]; then
    echo performance > /sys/class/devfreq/fdab0000.npu/governor
    echo "✅ NPU governor: performance"
else
    echo "⚠️  无法设置 NPU governor"
fi
echo ""

echo "3️⃣ 设置 NPU 最大频率"
echo "----------------------------------------"
if [ -w /sys/class/devfreq/fdab0000.npu/max_freq ]; then
    echo 1000000000 > /sys/class/devfreq/fdab0000.npu/max_freq
    echo "✅ NPU max_freq: 1000 MHz"
else
    echo "⚠️  无法设置 NPU max_freq"
fi
echo ""

echo "4️⃣ 验证设置"
echo "----------------------------------------"
echo "CPU 模式:"
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo ""
echo "NPU 模式:"
cat /sys/class/devfreq/fdab0000.npu/governor 2>/dev/null || echo "无法读取"
echo "NPU 频率:"
npu_freq=$(cat /sys/class/devfreq/fdab0000.npu/cur_freq 2>/dev/null)
if [ ! -z "$npu_freq" ]; then
    echo "  $((npu_freq / 1000000)) MHz"
else
    echo "  无法读取"
fi
echo ""

echo "=========================================="
echo "✅ 优化完成！"
echo "=========================================="
echo ""
echo "重启 YOLO 节点以应用优化："
echo "  1. Ctrl+C 停止当前节点"
echo "  2. ros2 launch yolov11n_rknn yolo_detection.launch.py"
echo ""
echo "预期性能提升："
echo "  - FPS: 17 → 30-50 FPS"
echo "  - 推理时间: < 30ms"
echo ""
echo "⚠️  注意：这些设置在重启后会失效"
echo "   如需永久生效，请参考："
echo "   https://github.com/rockchip-linux/rknn-toolkit2"
echo "=========================================="
