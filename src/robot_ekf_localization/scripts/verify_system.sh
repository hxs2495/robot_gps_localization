#!/bin/bash
# GPS融合定位系统快速验证脚本

echo "=========================================="
echo "GPS融合定位系统验证脚本"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_topic() {
    local topic=$1
    local description=$2
    echo -n "检查 $description ($topic)... "

    if timeout 2 ros2 topic echo $topic --once > /dev/null 2>&1; then
        echo -e "${GREEN}✓ 正常${NC}"
        return 0
    else
        echo -e "${RED}✗ 无数据${NC}"
        return 1
    fi
}

check_node() {
    local node=$1
    local description=$2
    echo -n "检查 $description ($node)... "

    if ros2 node list 2>/dev/null | grep -q $node; then
        echo -e "${GREEN}✓ 运行中${NC}"
        return 0
    else
        echo -e "${RED}✗ 未运行${NC}"
        return 1
    fi
}

check_sim_time() {
    local node=$1
    local description=$2
    local expected=$3

    if ! ros2 node list 2>/dev/null | grep -Fxq "$node"; then
        echo -e "${YELLOW}⚠ 跳过${NC} $description ($node 未运行)"
        return
    fi

    local value
    value=$(ros2 param get "$node" use_sim_time 2>/dev/null)
    if echo "$value" | grep -qi "$expected"; then
        echo -e "${GREEN}✓${NC} $description use_sim_time=$expected"
    else
        echo -e "${RED}✗${NC} $description 时钟模式不一致，期望 use_sim_time=$expected"
    fi
}

echo "1. 检查融合输入"
echo "-------------------"
check_topic "/Odometry" "FAST_LIO局部里程计"
check_topic "/odometry/gps" "GPS转换里程计"
echo ""

echo "2. 检查融合定位节点"
echo "-------------------"
check_node "ekf_filter_node_local" "局部EKF节点"
check_node "ekf_filter_node_global" "全局EKF节点"
check_node "gps_global_odometry_node" "GPS全局坐标转换节点"
echo ""

echo "3. 检查rosbag时钟一致性"
echo "-------------------"
if ros2 topic list 2>/dev/null | grep -Fxq "/clock"; then
    echo -e "${GREEN}✓ 检测到 /clock，按rosbag回放模式检查${NC}"
    expected_sim_time="true"
else
    echo -e "${GREEN}✓ 未检测到 /clock，按实时传感器模式检查${NC}"
    expected_sim_time="false"
fi
check_sim_time "/laser_mapping" "FAST_LIO" "$expected_sim_time"
check_sim_time "/gpgga_filter_node" "GPS过滤节点" "$expected_sim_time"
check_sim_time "/gps_to_utm_odometry_node" "GPS转UTM节点" "$expected_sim_time"
check_sim_time "/gps_global_odometry_node" "GPS全局转换节点" "$expected_sim_time"
check_sim_time "/ekf_filter_node_local" "局部EKF" "$expected_sim_time"
check_sim_time "/ekf_filter_node_global" "全局EKF" "$expected_sim_time"
echo ""

echo "4. 检查输出数据"
echo "-------------------"
check_topic "/odometry/local" "局部里程计输出"
check_topic "/odometry/global" "全局里程计输出"
echo ""

echo "5. 检查TF树"
echo "-------------------"
echo -n "检查 map->odom 变换... "
if timeout 2 ros2 run tf2_ros tf2_echo map odom > /dev/null 2>&1; then
    echo -e "${GREEN}✓ 正常${NC}"
else
    echo -e "${RED}✗ 无变换${NC}"
fi

echo -n "检查 odom->base_footprint 变换... "
if timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint > /dev/null 2>&1; then
    echo -e "${GREEN}✓ 正常${NC}"
else
    echo -e "${RED}✗ 无变换${NC}"
fi

echo "6. GPS质量检查（可选原始数据）"
echo "-------------------"
if timeout 2 ros2 topic echo /fix --once > /tmp/gps_check.txt 2>&1; then
    echo -e "${GREEN}GPS数据接收正常${NC}"

    # 提取GPS信息
    lat=$(grep "latitude:" /tmp/gps_check.txt | awk '{print $2}')
    lon=$(grep "longitude:" /tmp/gps_check.txt | awk '{print $2}')
    alt=$(grep "altitude:" /tmp/gps_check.txt | awk '{print $2}')
    status=$(grep "status:" /tmp/gps_check.txt | tail -1 | awk '{print $2}')

    echo "  位置: ($lat, $lon, $alt)"
    echo "  状态: $status"

    if [ "$status" -ge "0" ]; then
        echo -e "  ${GREEN}GPS定位正常${NC}"
    else
        echo -e "  ${YELLOW}GPS无定位信号${NC}"
    fi

else
    echo -e "${RED}无法接收GPS数据${NC}"
fi
echo ""

echo "=========================================="
echo "验证完成"
echo "=========================================="
echo ""
echo "建议操作："
echo "  - 查看实时定位: ros2 topic echo /odometry/global"
echo "  - 可视化TF树: ros2 run tf2_tools view_frames"
echo "  - 启动RViz查看: ros2 run rviz2 rviz2"
echo "  - 查看详细文档: cat ~/project/robot_gps_localization/design/2026-06-18-1500-GPS激光里程计融合定位系统.md"
echo ""
