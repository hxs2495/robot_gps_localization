#!/bin/bash
# GPS融合定位系统依赖安装脚本

echo "=========================================="
echo "安装robot_localization依赖"
echo "=========================================="

# 检测ROS2版本
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置"
    echo "请先运行: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "检测到ROS2发行版: $ROS_DISTRO"

# 更新包列表
echo "更新包列表..."
sudo apt update

# 安装robot_localization
echo "安装robot_localization..."
sudo apt install -y ros-${ROS_DISTRO}-robot-localization

# 安装其他依赖
echo "安装其他依赖..."
sudo apt install -y \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rviz2

# 检查安装
echo ""
echo "检查安装结果..."
if ros2 pkg list | grep -q robot_localization; then
    echo "✓ robot_localization 安装成功"
else
    echo "✗ robot_localization 安装失败"
    exit 1
fi

echo ""
echo "=========================================="
echo "依赖安装完成！"
echo "=========================================="
echo ""
echo "接下来请编译功能包："
echo "  cd ~/project/robot_gps_localization"
echo "  colcon build --packages-select robot_ekf_localization"
echo "  source install/setup.bash"
echo ""
