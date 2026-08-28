#!/bin/bash

# GPS定位bag回放脚本（带时钟同步）
# 关键参数：--clock 发布/clock话题实现时间同步

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  GPS定位系统 - Bag回放${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}重要说明：${NC}"
echo "  --clock: 发布/clock话题，确保所有节点使用bag文件的时间戳"
echo "  这是解决map坐标系数据显示异常的关键！"
echo ""

source install/setup.bash

BAG_FILE="robot_data/all-data-8-23-4/"

if [ ! -d "$BAG_FILE" ]; then
    echo -e "${RED}❌ 错误: bag文件不存在: $BAG_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}正在回放bag文件...${NC}"
echo -e "文件: $BAG_FILE"
echo ""
echo -e "${YELLOW}提示: 使用Ctrl+C停止回放${NC}"
echo ""

# 关键：使用--clock参数回放，实现时间同步
ros2 bag play "$BAG_FILE" --clock

# 可选参数：
# --rate 0.5    # 0.5倍速回放（慢速）
# --rate 2.0    # 2倍速回放（快速）
# --loop        # 循环回放
# --start-offset 10  # 从第10秒开始回放