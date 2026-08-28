#!/bin/bash
###############################################################################
# GPS Localization Test Script
# 该脚本用于测试GPS融合定位系统
###############################################################################

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}GPS融合定位系统测试${NC}"
echo -e "${BLUE}========================================${NC}"

# 设置工作目录
WORKSPACE_DIR="$HOME/project/robot_gps_localization"
cd "$WORKSPACE_DIR"

# Source ROS2环境
echo -e "\n${YELLOW}[1/4] 加载ROS2环境...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查bag文件
BAG_FILE="$WORKSPACE_DIR/robot_data/all-data-8-23-1"
if [ ! -d "$BAG_FILE" ]; then
    echo -e "${RED}错误: 找不到bag文件: $BAG_FILE${NC}"
    echo -e "${YELLOW}请确保bag文件存在${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Bag文件存在${NC}"

# 显示bag信息
echo -e "\n${YELLOW}[2/4] Bag文件信息:${NC}"
ros2 bag info "$BAG_FILE" | grep -E "Duration|Messages|Topic.*/(fix|imu|Odometry)"

# 启动选项
echo -e "\n${YELLOW}[3/4] 选择启动方式:${NC}"
echo "1) 启动定位系统 + 自动播放bag文件 (推荐)"
echo "2) 仅启动定位系统 (需要手动播放bag或连接真实传感器)"
echo "3) 仅播放bag文件"
read -p "请选择 [1-3]: " choice

case $choice in
    1)
        echo -e "\n${GREEN}启动定位系统 + bag文件播放...${NC}"
        echo -e "${YELLOW}按Ctrl+C停止${NC}\n"
        sleep 2
        ros2 launch robot_ekf_localization gps_localization_with_bag.launch.py
        ;;
    2)
        echo -e "\n${GREEN}仅启动定位系统...${NC}"
        echo -e "${YELLOW}提示: 请在另一个终端运行以下命令播放bag文件:${NC}"
        echo -e "${BLUE}ros2 bag play $BAG_FILE --clock${NC}\n"
        sleep 2
        ros2 launch robot_ekf_localization gps_localization.launch.py use_sim_time:=true
        ;;
    3)
        echo -e "\n${GREEN}仅播放bag文件...${NC}"
        echo -e "${YELLOW}按Ctrl+C停止${NC}\n"
        sleep 2
        ros2 bag play "$BAG_FILE" --clock
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac
