#!/usr/bin/env python3
"""
GPS-LiDAR Fusion Localization Launch File
========================================
该launch文件启动robot_localization的完整融合定位系统：
1. 局部EKF：融合激光里程计(FAST_LIO)和IMU，发布odom->base_footprint变换
2. NavSat转换：将GPS数据转换为本地笛卡尔坐标系的里程计消息
3. 全局EKF：融合所有传感器（里程计+IMU+GPS），发布map->odom变换

坐标系关系：map -> odom -> base_footprint -> base_link
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
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')

    # Launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use rosbag /clock by default; set false for live sensors'
    )

    # 局部EKF节点 - 融合里程计和IMU，输出odom->base_footprint变换
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_local',
        output='screen',
        parameters=[
            ekf_local_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/local')
        ]
    )



    return LaunchDescription([
        use_sim_time_arg,
        ekf_local_node,

    ])
