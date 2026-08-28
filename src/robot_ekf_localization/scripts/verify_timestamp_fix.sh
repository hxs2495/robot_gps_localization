#!/bin/bash
# GPS时间戳修正验证脚本

echo "=========================================="
echo "GPS时间戳修正验证脚本"
echo "=========================================="
echo ""

# 检查时间戳修正节点是否运行
echo "1. 检查时间戳修正节点..."
if ros2 node list | grep -q "timestamp_corrector"; then
    echo "   ✅ timestamp_corrector节点正在运行"
else
    echo "   ❌ timestamp_corrector节点未运行"
    echo "   请先启动定位系统："
    echo "   ros2 launch robot_ekf_localization gps_localization.launch.py"
    exit 1
fi

echo ""
echo "2. 检查话题是否存在..."

# 检查/fix话题
if ros2 topic list | grep -q "^/fix$"; then
    echo "   ✅ /fix 话题存在"
else
    echo "   ❌ /fix 话题不存在（需要rosbag播放）"
fi

# 检查/fix_corrected话题
if ros2 topic list | grep -q "^/fix_corrected$"; then
    echo "   ✅ /fix_corrected 话题存在"
else
    echo "   ❌ /fix_corrected 话题不存在"
    exit 1
fi

echo ""
echo "3. 获取时间戳..."

# 获取原始GPS时间戳
echo "   正在读取 /fix 时间戳..."
fix_sec=$(timeout 3 ros2 topic echo /fix --field header.stamp.sec --once 2>/dev/null)
if [ -z "$fix_sec" ]; then
    echo "   ⚠️  无法读取 /fix 时间戳（可能无数据）"
else
    echo "   原始GPS时间:    $fix_sec 秒"
fi

# 获取修正后GPS时间戳
echo "   正在读取 /fix_corrected 时间戳..."
corrected_sec=$(timeout 3 ros2 topic echo /fix_corrected --field header.stamp.sec --once 2>/dev/null)
if [ -z "$corrected_sec" ]; then
    echo "   ❌ 无法读取 /fix_corrected 时间戳"
    exit 1
else
    echo "   修正GPS时间:    $corrected_sec 秒"
fi

# 获取局部里程计时间戳
echo "   正在读取 /odometry/local 时间戳..."
local_sec=$(timeout 3 ros2 topic echo /odometry/local --field header.stamp.sec --once 2>/dev/null)
if [ -z "$local_sec" ]; then
    echo "   ⚠️  无法读取 /odometry/local 时间戳"
else
    echo "   局部里程计时间: $local_sec 秒"
fi

echo ""
echo "4. 分析时间戳差异..."

if [ ! -z "$fix_sec" ] && [ ! -z "$corrected_sec" ]; then
    diff_before=$((corrected_sec - fix_sec))
    echo "   修正前后差异: $diff_before 秒 (约 $((diff_before / 86400)) 天)"

    if [ $diff_before -gt 1000000 ]; then
        echo "   ✅ 时间戳已成功修正（差异显著）"
    else
        echo "   ⚠️  时间戳差异较小，可能数据本身就是同步的"
    fi
fi

if [ ! -z "$corrected_sec" ] && [ ! -z "$local_sec" ]; then
    diff_sync=$((corrected_sec - local_sec))
    diff_sync_abs=${diff_sync#-}  # 取绝对值

    echo "   修正后GPS vs 局部里程计: $diff_sync 秒"

    if [ $diff_sync_abs -le 1 ]; then
        echo "   ✅ 时间戳同步良好（差异 ≤1秒）"
    elif [ $diff_sync_abs -le 10 ]; then
        echo "   ⚠️  时间戳有轻微偏差（1-10秒）"
    else
        echo "   ❌ 时间戳仍然不同步（差异 >10秒）"
    fi
fi

echo ""
echo "5. 检查TF树..."

# 检查map到odom变换
echo "   正在检查 map → odom 变换..."
if timeout 3 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "Translation:"; then
    echo "   ✅ map → odom 变换存在"
    echo "   ✅ 全局EKF已成功发布TF变换"
else
    echo "   ❌ map → odom 变换不存在"
    echo "   可能需要等待几秒让系统初始化，或检查EKF日志"
fi

echo ""
echo "=========================================="
echo "验证完成！"
echo ""
echo "如果所有检查都通过，可以在RViz中："
echo "1. 设置 Fixed Frame 为 'map'"
echo "2. 添加 Odometry 显示："
echo "   - /odometry/local"
echo "   - /odometry/gps"
echo "   - /odometry/global"
echo "3. 应该能看到所有轨迹正常显示"
echo "=========================================="
