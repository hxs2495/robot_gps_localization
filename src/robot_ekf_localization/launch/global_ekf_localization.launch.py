#!/usr/bin/env python3
"""Launch the global EKF for LiDAR-GPS fusion."""

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

    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
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
                description="odom坐标系中的全局融合里程计输出",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="由全局EKF发布GPS修正后的odom->base_footprint",
            ),
            LogInfo(
                msg=(
                    "全局EKF要求 /odometry/gps.header.frame_id=map，"
                    "map->odom固定为单位静态变换；请勿再发布同名TF"
                )
            ),
            static_map_to_odom,
            global_ekf,
        ]
    )
