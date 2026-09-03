#!/usr/bin/env python3
"""Launch the NavSatFix-to-global-odometry conversion chain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context, default_config_file):
    requested_config = (
        LaunchConfiguration("config_file").perform(context).strip()
    )
    config_file = requested_config or default_config_file
    actions = []

    if not os.path.isfile(config_file):
        actions.append(
            LogInfo(
                msg=(
                    f"[WARN] 运行参数文件不存在: {config_file}；"
                    f"回退到包内默认配置: {default_config_file}"
                )
            )
        )
        config_file = default_config_file

    common_overrides = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    parameter_files = [config_file] if os.path.isfile(config_file) else []

    actions.append(
        LogInfo(
            msg=(
                "GPS转换使用配置: "
                f"{config_file if parameter_files else '节点内置默认参数'}"
            )
        )
    )

    actions.extend(
        [
            Node(
                package="robot_odom_transform",
                executable="gpgga_filter_node",
                name="gpgga_filter_node",
                output="screen",
                parameters=parameter_files
                + [
                    common_overrides,
                    {
                        "input_topic": LaunchConfiguration("gps_topic"),
                        "output_topic": LaunchConfiguration(
                            "filtered_gps_topic"
                        ),
                    },
                ],
            ),
            Node(
                package="robot_odom_transform",
                executable="gps_to_utm_odometry_node",
                name="gps_to_utm_odometry_node",
                output="screen",
                parameters=parameter_files
                + [
                    common_overrides,
                    {
                        "gps_topic": LaunchConfiguration("filtered_gps_topic"),
                        "orient_topic": LaunchConfiguration(
                            "orientation_topic"
                        ),
                        "odom_topic": LaunchConfiguration("utm_odom_topic"),
                    },
                ],
            ),
            Node(
                package="robot_odom_transform",
                executable="gps_global_odometry_node",
                name="gps_global_odometry_node",
                output="screen",
                parameters=parameter_files
                + [
                    common_overrides,
                    {
                        "input_odom_topic": LaunchConfiguration(
                            "utm_odom_topic"
                        ),
                        "output_odom_topic": LaunchConfiguration(
                            "output_odom_topic"
                        ),
                        "path_topic": LaunchConfiguration(
                            "output_path_topic"
                        ),
                        "global_frame_id": LaunchConfiguration(
                            "global_frame_id"
                        ),
                        "child_frame_id": LaunchConfiguration(
                            "child_frame_id"
                        ),
                    },
                ],
            ),
        ]
    )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("robot_odom_transform")
    default_config_file = os.path.join(
        package_share, "config", "gps_transform.defaults.yaml"
    )
    description_share = get_package_share_directory("robot_description")
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_share, "launch", "robot_description.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "urdf_file": LaunchConfiguration("urdf_file"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("publish_robot_description")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="运行参数YAML；传感器安装外参必须只写在URDF中",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用/clock仿真时间；回放rosbag时设为true",
            ),
            DeclareLaunchArgument(
                "publish_robot_description",
                default_value="true",
                description="独立启动时发布URDF静态传感器TF",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value=os.path.join(
                    description_share, "urdf", "robot.urdf"
                ),
                description="传感器安装外参的唯一URDF文件",
            ),
            DeclareLaunchArgument(
                "gps_topic", default_value="/fix", description="原始NavSatFix话题"
            ),
            DeclareLaunchArgument(
                "filtered_gps_topic",
                default_value="/fix/filter",
                description="过滤后的NavSatFix话题",
            ),
            DeclareLaunchArgument(
                "orientation_topic",
                default_value="/imu_orientation",
                description="双天线GNSS或IMU航向话题",
            ),
            DeclareLaunchArgument(
                "utm_odom_topic",
                default_value="/utm/gps",
                description="UTM坐标系下的GPS天线里程计",
            ),
            DeclareLaunchArgument(
                "output_odom_topic",
                default_value="/odometry/gps",
                description="机器人全局坐标系下的GPS基座里程计",
            ),
            DeclareLaunchArgument(
                "output_path_topic",
                default_value="/gps_path",
                description="转换后的GPS路径话题",
            ),
            DeclareLaunchArgument(
                "global_frame_id",
                default_value="map",
                description="GPS绝对观测坐标系；全局EKF融合时必须为map",
            ),
            DeclareLaunchArgument(
                "child_frame_id",
                default_value="base_footprint",
                description="由TF转换得到的统一融合参考坐标系",
            ),
            robot_description,
            OpaqueFunction(
                function=_launch_nodes,
                args=[default_config_file],
            ),
        ]
    )
