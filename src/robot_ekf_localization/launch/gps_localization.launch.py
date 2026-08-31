#!/usr/bin/env python3
"""Launch local and global EKFs for LiDAR-GPS fusion."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_ekf_localization")
    default_local_config = os.path.join(
        package_share, "config", "ekf_local.yaml"
    )
    default_global_config = os.path.join(
        package_share, "config", "ekf_global.yaml"
    )

    local_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[
            LaunchConfiguration("local_config_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom0": LaunchConfiguration("lio_odom_topic"),
                "publish_tf": LaunchConfiguration("publish_local_tf"),
            },
        ],
        remappings=[
            ("odometry/filtered", LaunchConfiguration("local_odom_topic")),
        ],
    )

    global_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="screen",
        parameters=[
            LaunchConfiguration("global_config_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom0": LaunchConfiguration("local_odom_topic"),
                "odom1": LaunchConfiguration("gps_odom_topic"),
                "publish_tf": LaunchConfiguration("publish_global_tf"),
            },
        ],
        remappings=[
            ("odometry/filtered", LaunchConfiguration("global_odom_topic")),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="是否使用/clock仿真时间；实时传感器运行时设为false",
            ),
            DeclareLaunchArgument(
                "local_config_file",
                default_value=default_local_config,
                description="局部EKF参数文件",
            ),
            DeclareLaunchArgument(
                "global_config_file",
                default_value=default_global_config,
                description="全局EKF参数文件",
            ),
            DeclareLaunchArgument(
                "lio_odom_topic",
                default_value="/Odometry",
                description="FAST_LIO局部里程计输入",
            ),
            DeclareLaunchArgument(
                "gps_odom_topic",
                default_value="/odometry/gps",
                description="转换到map坐标系的GPS绝对里程计输入",
            ),
            DeclareLaunchArgument(
                "local_odom_topic",
                default_value="/odometry/local",
                description="局部EKF输出",
            ),
            DeclareLaunchArgument(
                "global_odom_topic",
                default_value="/odometry/global",
                description="GPS修正后的全局融合里程计输出",
            ),
            DeclareLaunchArgument(
                "publish_local_tf",
                default_value="true",
                description="由局部EKF发布odom->base_footprint",
            ),
            DeclareLaunchArgument(
                "publish_global_tf",
                default_value="true",
                description="由全局EKF发布map->odom",
            ),
            local_ekf,
            global_ekf,
        ]
    )
