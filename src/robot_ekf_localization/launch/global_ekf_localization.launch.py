#!/usr/bin/env python3
"""启动 FAST_LIO + GPS 的全局 EKF。

前置条件:
  /odometry/local - odom 坐标系中的连续局部里程计
  /odometry/gps   - map 坐标系中的 GPS 绝对里程计
  odom -> base_footprint TF 由局部 EKF 唯一发布

输出:
  /odometry/global - map 坐标系中的全局融合里程计
  map -> odom TF   - GPS 对局部里程计漂移的动态校正
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_ekf_localization")
    default_config = os.path.join(
        package_share, "config", "ekf_global.yaml"
    )

    global_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom0": LaunchConfiguration("local_odom_topic"),
                "odom1": LaunchConfiguration("gps_odom_topic"),
                "publish_tf": LaunchConfiguration("publish_tf"),
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
                description="默认使用rosbag的/clock；实时传感器运行时设为false",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="全局EKF参数文件",
            ),
            DeclareLaunchArgument(
                "local_odom_topic",
                default_value="/odometry/local",
                description="odom坐标系中的局部EKF输入",
            ),
            DeclareLaunchArgument(
                "gps_odom_topic",
                default_value="/odometry/gps",
                description="map坐标系中的GPS绝对里程计输入",
            ),
            DeclareLaunchArgument(
                "global_odom_topic",
                default_value="/odometry/global",
                description="map坐标系中的全局融合里程计输出",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="由全局EKF唯一发布动态map->odom",
            ),
            LogInfo(
                msg=(
                    "全局EKF要求 /odometry/gps.header.frame_id=map，"
                    "map->odom是动态漂移校正，不能同时发布静态变换"
                )
            ),
            global_ekf,
        ]
    )
