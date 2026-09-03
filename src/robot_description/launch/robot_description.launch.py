#!/usr/bin/env python3
"""Publish the canonical URDF as the only source of static sensor TFs."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_publisher(context):
    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    with open(urdf_file, "r", encoding="utf-8") as stream:
        robot_description = stream.read()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("robot_description")
    default_urdf = os.path.join(package_share, "urdf", "robot.urdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_file",
                default_value=default_urdf,
                description="唯一的机器人及传感器静态外参URDF",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="回放rosbag时设为true",
            ),
            OpaqueFunction(function=_create_publisher),
        ]
    )
