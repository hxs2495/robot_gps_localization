#!/usr/bin/env python3
"""Launch the local EKF for continuous LiDAR odometry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_ekf_localization")
    default_config = os.path.join(package_share, "config", "ekf_local.yaml")

    local_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom0": LaunchConfiguration("lio_odom_topic"),
                "publish_tf": LaunchConfiguration("publish_tf"),
            },
        ],
        remappings=[
            ("odometry/filtered", LaunchConfiguration("local_odom_topic")),
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
                "config_file",
                default_value=default_config,
                description="局部EKF参数文件",
            ),
            DeclareLaunchArgument(
                "lio_odom_topic",
                default_value="/Odometry",
                description="FAST-LIO局部里程计输入",
            ),
            DeclareLaunchArgument(
                "local_odom_topic",
                default_value="/odometry/local",
                description="局部EKF输出",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="由局部EKF发布odom->base_footprint",
            ),
            local_ekf,
        ]
    )
