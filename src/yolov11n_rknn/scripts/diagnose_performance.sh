#!/bin/bash
# YOLOv11n RKNN 性能诊断脚本

echo "=========================================="
echo "YOLOv11n RKNN 性能诊断"
echo "=========================================="
echo ""

# 1. 检查 CPU 频率模式
echo "1️⃣ CPU 频率模式检查"
echo "----------------------------------------"
for cpu in /sys/devices/system/cpu/cpu[0-7]/cpufreq/scaling_governor; do
    governor=$(cat $cpu)
    echo "CPU $(basename $(dirname $cpu)): $governor"
done
echo ""

# 2. 检查 NPU 频率
echo "2️⃣ NPU 频率检查"
echo "----------------------------------------"
if [ -d /sys/class/devfreq/fdab0000.npu ]; then
    npu_governor=$(cat /sys/class/devfreq/fdab0000.npu/governor 2>/dev/null)
    npu_cur_freq=$(cat /sys/class/devfreq/fdab0000.npu/cur_freq 2>/dev/null)
    npu_min_freq=$(cat /sys/class/devfreq/fdab0000.npu/min_freq 2>/dev/null)
    npu_max_freq=$(cat /sys/class/devfreq/fdab0000.npu/max_freq 2>/dev/null)

    echo "NPU 调频器: $npu_governor"
    echo "当前频率: $((npu_cur_freq / 1000000)) MHz"
    echo "最小频率: $((npu_min_freq / 1000000)) MHz"
    echo "最大频率: $((npu_max_freq / 1000000)) MHz"
else
    echo "⚠️  无法访问 NPU 频率信息"
fi
echo ""

# 3. 检查摄像头能力
echo "3️⃣ 摄像头能力检查"
echo "----------------------------------------"
for video in /dev/video*; do
    if [ -e "$video" ]; then
        echo "设备: $video"
        v4l2-ctl --device=$video --list-formats-ext 2>/dev/null | grep -A 5 "Pixel Format" | head -20 || echo "  无法获取信息"
        echo ""
    fi
done

# 4. 检查系统负载
echo "4️⃣ 系统负载"
echo "----------------------------------------"
uptime
echo ""

# 5. 检查内存使用
echo "5️⃣ 内存使用"
echo "----------------------------------------"
free -h
echo ""

# 6. 检查 NPU 驱动状态
echo "6️⃣ NPU 驱动状态"
echo "----------------------------------------"
lsmod | grep rknpu || echo "⚠️  RKNPU 模块未加载"
echo ""

# 7. 推荐优化措施
echo "7️⃣ 性能优化建议"
echo "------------------------------------------"
echo "如果CPU不是performance模式："
echo "  sudo sh -c 'echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor'"
echo ""
echo "如果NPU不是performance模式："
echo "  sudo sh -c 'echo performance > /sys/class/devfreq/fdab0000.npu/governor'"
echo ""
echo "临时设置NPU最大频率："
echo "  sudo sh -c 'echo 1000000000 > /sys/class/devfreq/fdab0000.npu/max_freq'"
echo ""
echo "禁用可视化显示（可节省CPU）："
echo "  ros2 launch yolov11n_rknn yolo_detection.launch.py show_detection:=false"
echo ""
echo "检查摄像头是否支持高帧率："
echo "  v4l2-ctl --device=/dev/video0 --list-formats-ext"
echo "=========================================="
