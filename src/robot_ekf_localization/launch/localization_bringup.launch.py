#!/usr/bin/env python3
"""Start the TF model, FAST-LIO, GPS conversion, and the two-EKF fusion chain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package_name, launch_file, arguments, condition=None):
    package_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", launch_file)
        ),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    description_share = get_package_share_directory("robot_description")
    fast_lio_share = get_package_share_directory("fast_lio")
    localization_share = get_package_share_directory(
        "robot_ekf_localization"
    )

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
            "filtered_gps_topic": LaunchConfiguration(
                "filtered_gps_topic"
            ),
            "orientation_topic": LaunchConfiguration("orientation_topic"),
            "utm_odom_topic": LaunchConfiguration("utm_odom_topic"),
            "output_odom_topic": LaunchConfiguration("gps_odom_topic"),
            "output_path_topic": LaunchConfiguration("gps_path_topic"),
            "global_frame_id": LaunchConfiguration("map_frame"),
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
            "local_config_file": LaunchConfiguration("local_config_file"),
            "global_config_file": LaunchConfiguration(
                "global_config_file"
            ),
            "lio_sensor_odom_topic": LaunchConfiguration(
                "lio_sensor_odom_topic"
            ),
            "lio_base_odom_topic": LaunchConfiguration("lio_base_odom_topic"),
            "lio_sensor_frame": LaunchConfiguration("lio_sensor_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "gps_odom_topic": LaunchConfiguration("gps_odom_topic"),
            "smoothed_gps_odom_topic": LaunchConfiguration(
                "smoothed_gps_odom_topic"
            ),
            "local_odom_topic": LaunchConfiguration("local_odom_topic"),
            "global_odom_topic": LaunchConfiguration("global_odom_topic"),
        },
    )

    paths = _include(
        "robot_fusion_path",
        "fusion_path.launch.py",
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "odom_topic": LaunchConfiguration("global_odom_topic"),
            "path_topic": LaunchConfiguration("global_path_topic"),
            "path_frame_id": LaunchConfiguration("path_frame_id"),
            "local_odom_topic": LaunchConfiguration("local_odom_topic"),
            "local_path_topic": LaunchConfiguration("local_path_topic"),
            "local_path_frame_id": LaunchConfiguration("path_frame_id"),
            "max_path_size": LaunchConfiguration("path_max_size"),
            "min_distance": LaunchConfiguration("path_min_distance"),
        },
        condition=IfCondition(LaunchConfiguration("publish_paths")),
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
                "local_config_file",
                default_value=os.path.join(
                    localization_share, "config", "ekf_local.yaml"
                ),
                description="局部EKF参数文件",
            ),
            DeclareLaunchArgument(
                "global_config_file",
                default_value=os.path.join(
                    localization_share, "config", "ekf_global.yaml"
                ),
                description="全局EKF与GPS恢复状态机参数文件",
            ),
            DeclareLaunchArgument(
                "gps_topic", default_value="/fix", description="NavSatFix输入"
            ),
            DeclareLaunchArgument(
                "filtered_gps_topic",
                default_value="/fix/filter",
                description="GPS有效性过滤后的NavSatFix话题",
            ),
            DeclareLaunchArgument(
                "orientation_topic",
                default_value="/imu_orientation",
                description="GPS/双天线航向输入",
            ),
            DeclareLaunchArgument(
                "utm_odom_topic",
                default_value="/utm/gps",
                description="UTM坐标下的GPS天线里程计",
            ),
            DeclareLaunchArgument(
                "gps_odom_topic",
                default_value="/odometry/gps",
                description="换算到base_frame的GPS全局里程计",
            ),
            DeclareLaunchArgument(
                "gps_path_topic",
                default_value="/gps_path",
                description="换算后的GPS Path话题",
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
                "local_odom_topic",
                default_value="/odometry/local",
                description="局部EKF输出",
            ),
            DeclareLaunchArgument(
                "smoothed_gps_odom_topic",
                default_value="/odometry/gps/smoothed",
                description="GPS消失/恢复状态机输出",
            ),
            DeclareLaunchArgument(
                "global_odom_topic",
                default_value="/odometry/global",
                description="全局融合里程计输出",
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
                "map_frame",
                default_value="map",
                description="GPS绝对观测frame；必须与全局EKF配置一致",
            ),
            DeclareLaunchArgument(
                "publish_paths",
                default_value="true",
                description="是否随统一入口发布local/global Path",
            ),
            DeclareLaunchArgument(
                "local_path_topic",
                default_value="/local_path",
                description="局部融合Path话题",
            ),
            DeclareLaunchArgument(
                "global_path_topic",
                default_value="/global_path",
                description="全局融合Path话题",
            ),
            DeclareLaunchArgument(
                "path_frame_id",
                default_value="",
                description="Path frame；留空时继承对应Odometry frame",
            ),
            DeclareLaunchArgument(
                "path_max_size",
                default_value="10000",
                description="local/global Path最多保留点数，0表示不限制",
            ),
            DeclareLaunchArgument(
                "path_min_distance",
                default_value="0.05",
                description="Path相邻采样点最小距离(m)",
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
            paths,
        ]
    )
