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
