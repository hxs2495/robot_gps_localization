#!/usr/bin/env python3
"""Replay a bag with the full localization stack in an isolated ROS domain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _start_bag_player(context):
    bag_path = os.path.abspath(
        os.path.expanduser(LaunchConfiguration("bag_path").perform(context))
    )
    if not os.path.isdir(bag_path):
        raise RuntimeError(f"rosbag目录不存在: {bag_path}")

    rate = LaunchConfiguration("playback_rate").perform(context)
    player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
            "--rate",
            rate,
        ],
        name="localization_bag_player",
        output="screen",
    )
    stop_when_finished = RegisterEventHandler(
        OnProcessExit(
            target_action=player,
            on_exit=[EmitEvent(event=Shutdown(reason="rosbag回放结束"))],
        )
    )
    return [player, stop_when_finished]


def generate_launch_description():
    package_share = get_package_share_directory("robot_ekf_localization")
    fast_lio_share = get_package_share_directory("fast_lio")
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share, "launch", "localization_bringup.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "rviz": LaunchConfiguration("rviz"),
            "urdf_file": LaunchConfiguration("urdf_file"),
            "fast_lio_config_path": LaunchConfiguration(
                "fast_lio_config_path"
            ),
            "fast_lio_config_file": LaunchConfiguration(
                "fast_lio_config_file"
            ),
            "gps_config_file": LaunchConfiguration("gps_config_file"),
            "local_config_file": LaunchConfiguration("local_config_file"),
            "global_config_file": LaunchConfiguration(
                "global_config_file"
            ),
            "gps_topic": LaunchConfiguration("gps_topic"),
            "filtered_gps_topic": LaunchConfiguration(
                "filtered_gps_topic"
            ),
            "orientation_topic": LaunchConfiguration("orientation_topic"),
            "utm_odom_topic": LaunchConfiguration("utm_odom_topic"),
            "gps_odom_topic": LaunchConfiguration("gps_odom_topic"),
            "gps_path_topic": LaunchConfiguration("gps_path_topic"),
            "lio_sensor_odom_topic": LaunchConfiguration(
                "lio_sensor_odom_topic"
            ),
            "lio_base_odom_topic": LaunchConfiguration(
                "lio_base_odom_topic"
            ),
            "local_odom_topic": LaunchConfiguration("local_odom_topic"),
            "smoothed_gps_odom_topic": LaunchConfiguration(
                "smoothed_gps_odom_topic"
            ),
            "global_odom_topic": LaunchConfiguration("global_odom_topic"),
            "lio_sensor_frame": LaunchConfiguration("lio_sensor_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "map_frame": LaunchConfiguration("map_frame"),
            "publish_paths": LaunchConfiguration("publish_paths"),
            "local_path_topic": LaunchConfiguration("local_path_topic"),
            "global_path_topic": LaunchConfiguration("global_path_topic"),
            "path_max_size": LaunchConfiguration("path_max_size"),
            "path_min_distance": LaunchConfiguration("path_min_distance"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="robot_data/all-data-8-23-4",
                description="要回放的rosbag目录",
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="1.0",
                description="rosbag回放倍率；定位验收建议先用1.0",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="42",
                description="隔离Gazebo、实时驱动及其他bag播放器的ROS域",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="false", description="是否启动RViz"
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value=os.path.join(
                    get_package_share_directory("robot_description"),
                    "urdf",
                    "robot.urdf",
                ),
                description="传感器安装外参的唯一URDF文件",
            ),
            DeclareLaunchArgument(
                "fast_lio_config_path",
                default_value=os.path.join(fast_lio_share, "config"),
                description="FAST-LIO配置目录",
            ),
            DeclareLaunchArgument(
                "fast_lio_config_file",
                default_value="mid360.yaml",
                description="FAST-LIO配置文件名",
            ),
            DeclareLaunchArgument(
                "gps_config_file",
                default_value="",
                description="GPS转换运行参数文件",
            ),
            DeclareLaunchArgument(
                "local_config_file",
                default_value=os.path.join(
                    package_share, "config", "ekf_local.yaml"
                ),
                description="局部EKF参数文件",
            ),
            DeclareLaunchArgument(
                "global_config_file",
                default_value=os.path.join(
                    package_share, "config", "ekf_global.yaml"
                ),
                description="全局EKF与GPS恢复参数文件",
            ),
            DeclareLaunchArgument(
                "gps_topic", default_value="/fix", description="NavSatFix输入"
            ),
            DeclareLaunchArgument(
                "filtered_gps_topic", default_value="/fix/filter"
            ),
            DeclareLaunchArgument(
                "orientation_topic", default_value="/imu_orientation"
            ),
            DeclareLaunchArgument("utm_odom_topic", default_value="/utm/gps"),
            DeclareLaunchArgument(
                "gps_odom_topic", default_value="/odometry/gps"
            ),
            DeclareLaunchArgument("gps_path_topic", default_value="/gps_path"),
            DeclareLaunchArgument(
                "lio_sensor_odom_topic", default_value="/Odometry"
            ),
            DeclareLaunchArgument(
                "lio_base_odom_topic", default_value="/odometry/lio/base"
            ),
            DeclareLaunchArgument(
                "local_odom_topic", default_value="/odometry/local"
            ),
            DeclareLaunchArgument(
                "smoothed_gps_odom_topic",
                default_value="/odometry/gps/smoothed",
            ),
            DeclareLaunchArgument(
                "global_odom_topic", default_value="/odometry/global"
            ),
            DeclareLaunchArgument(
                "lio_sensor_frame", default_value="livox_imu"
            ),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("publish_paths", default_value="true"),
            DeclareLaunchArgument(
                "local_path_topic", default_value="/local_path"
            ),
            DeclareLaunchArgument(
                "global_path_topic", default_value="/global_path"
            ),
            DeclareLaunchArgument("path_max_size", default_value="10000"),
            DeclareLaunchArgument("path_min_distance", default_value="0.05"),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            bringup,
            # Give robot_state_publisher and all subscribers time to discover
            # each other before the first recorded sensor messages arrive.
            TimerAction(
                period=2.0,
                actions=[OpaqueFunction(function=_start_bag_player)],
            ),
        ]
    )
