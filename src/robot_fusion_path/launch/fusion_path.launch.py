#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fusion_path_node = Node(
        package="robot_fusion_path",
        executable="fusion_path_node",
        name="fusion_path_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "path_topic": LaunchConfiguration("path_topic"),
                "path_frame_id": LaunchConfiguration("path_frame_id"),
                "local_odom_topic": LaunchConfiguration("local_odom_topic"),
                "local_path_topic": LaunchConfiguration("local_path_topic"),
                "local_path_frame_id": LaunchConfiguration(
                    "local_path_frame_id"
                ),
                "max_path_size": LaunchConfiguration("max_path_size"),
                "min_distance": LaunchConfiguration("min_distance"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="是否使用/clock仿真时间",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/global",
                description="全局融合后的里程计话题",
            ),
            DeclareLaunchArgument(
                "path_topic",
                default_value="/global_path",
                description="发布的全局Path话题",
            ),
            DeclareLaunchArgument(
                "path_frame_id",
                default_value="",
                description="全局Path坐标系；为空时使用Odometry的header.frame_id",
            ),
            DeclareLaunchArgument(
                "local_odom_topic",
                default_value="/odometry/local",
                description="局部融合后的里程计话题",
            ),
            DeclareLaunchArgument(
                "local_path_topic",
                default_value="/local_path",
                description="发布的局部Path话题",
            ),
            DeclareLaunchArgument(
                "local_path_frame_id",
                default_value="",
                description="局部Path坐标系；为空时使用Odometry的header.frame_id",
            ),
            DeclareLaunchArgument(
                "max_path_size",
                default_value="0",
                description="最多保留的路径点数量，0表示不限制",
            ),
            DeclareLaunchArgument(
                "min_distance",
                default_value="0.0",
                description="相邻路径点最小距离，0表示每帧都记录",
            ),
            fusion_path_node,
        ]
    )
