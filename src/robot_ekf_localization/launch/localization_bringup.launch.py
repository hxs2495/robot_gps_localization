#!/usr/bin/env python3
"""Start the TF model, FAST-LIO, GPS conversion, and the two-EKF fusion chain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package_name, launch_file, arguments):
    package_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", launch_file)
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    description_share = get_package_share_directory("robot_description")
    fast_lio_share = get_package_share_directory("fast_lio")

    robot_description = _include(
        "robot_description",
        "robot_description.launch.py",
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "urdf_file": LaunchConfiguration("urdf_file"),
        },
    )

    fast_lio = _include(
        "fast_lio",
        "mapping.launch.py",
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_robot_description": "false",
            "config_path": LaunchConfiguration("fast_lio_config_path"),
            "config_file": LaunchConfiguration("fast_lio_config_file"),
            "rviz": LaunchConfiguration("rviz"),
        },
    )

    gps_transform = _include(
        "robot_odom_transform",
        "start_gps_transform.launch.py",
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_robot_description": "false",
            "config_file": LaunchConfiguration("gps_config_file"),
            "gps_topic": LaunchConfiguration("gps_topic"),
            "orientation_topic": LaunchConfiguration("orientation_topic"),
            "child_frame_id": LaunchConfiguration("base_frame"),
        },
    )

    fusion = _include(
        "robot_ekf_localization",
        "gps_localization.launch.py",
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_robot_description": "false",
            "urdf_file": LaunchConfiguration("urdf_file"),
            "lio_sensor_odom_topic": LaunchConfiguration(
                "lio_sensor_odom_topic"
            ),
            "lio_base_odom_topic": LaunchConfiguration("lio_base_odom_topic"),
            "lio_sensor_frame": LaunchConfiguration("lio_sensor_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="rosbag回放设true，实时传感器设false",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value=os.path.join(
                    description_share, "urdf", "robot.urdf"
                ),
                description="机器人与全部传感器静态外参的唯一来源",
            ),
            DeclareLaunchArgument(
                "fast_lio_config_path",
                default_value=os.path.join(fast_lio_share, "config"),
                description="FAST-LIO配置目录（不再包含安装外参）",
            ),
            DeclareLaunchArgument(
                "fast_lio_config_file",
                default_value="mid360.yaml",
                description="FAST-LIO配置文件名",
            ),
            DeclareLaunchArgument(
                "gps_config_file",
                default_value="",
                description="GPS运行参数文件（不得包含传感器安装外参）",
            ),
            DeclareLaunchArgument(
                "gps_topic", default_value="/fix", description="NavSatFix输入"
            ),
            DeclareLaunchArgument(
                "orientation_topic",
                default_value="/imu_orientation",
                description="GPS/双天线航向输入",
            ),
            DeclareLaunchArgument(
                "lio_sensor_odom_topic",
                default_value="/Odometry",
                description="FAST-LIO原生IMU参考点里程计",
            ),
            DeclareLaunchArgument(
                "lio_base_odom_topic",
                default_value="/odometry/lio/base",
                description="转换到融合参考点后的激光里程计",
            ),
            DeclareLaunchArgument(
                "lio_sensor_frame",
                default_value="livox_imu",
                description="FAST-LIO状态对应的传感器frame",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="所有观测统一使用的融合参考frame",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="false", description="是否启动FAST-LIO RViz"
            ),
            # The description publisher is intentionally included once. All
            # downstream nodes wait for its static TFs instead of using numeric
            # calibration parameters from their YAML files.
            robot_description,
            fast_lio,
            gps_transform,
            fusion,
        ]
    )
