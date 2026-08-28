#!/usr/bin/env python3
"""
GPS Transform Only Launch File
===============================
该launch文件仅启动GPS转换节点，将/fix话题转换为/odometry/gps
用于在rviz2中可视化GPS数据

功能：
1. NavSat转换：将GPS数据(WGS84经纬度)转换为本地笛卡尔坐标系的里程计消息
2. 输出/odometry/gps话题，可在rviz2中显示

使用场景：
- 单独测试GPS转换功能
- 在rviz2中可视化GPS轨迹
- 调试GPS数据质量

前置条件：
- 需要/fix话题（GPS数据）
- 需要/livox/imu话题（IMU数据，用于姿态）
- 需要/Odometry话题（里程计数据，用于yaw角）

启动命令：
    ros2 launch robot_ekf_localization gps_transform_only.launch.py

在rviz2中查看：
    Fixed Frame: map
    添加 Odometry 显示，话题选择 /odometry/gps
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('robot_ekf_localization')

    # 配置文件路径
    navsat_config = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')

    # Launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # NavSat转换节点 - 将GPS转换为里程计消息
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            navsat_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # GPS输入
            ('/gps/fix', '/fix_filtered'),

            # IMU输入（用于姿态）
            ('/imu', '/livox/imu_filtered'),

            # 里程计输入（用于yaw角，因为use_odometry_yaw: true）
            # 直接使用FAST_LIO的输出
            ('/odometry/filtered', '/Odometry'),

            # GPS转换输出
            ('/odometry/gps', '/odometry/gps'),
            ('/gps/filtered', '/gps/filtered')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        navsat_transform_node,
    ])
