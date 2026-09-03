#!/usr/bin/env python3
"""Launch local and global EKFs for LiDAR-GPS fusion."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_ekf_localization")
    default_local_config = os.path.join(
        package_share, "config", "ekf_local.yaml"
    )
    default_global_config = os.path.join(
        package_share, "config", "ekf_global.yaml"
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

    lio_reference_transform = Node(
        package="robot_odom_transform",
        executable="odometry_tf_transform_node",
        name="lio_reference_transform",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": LaunchConfiguration("lio_sensor_odom_topic"),
                "output_topic": LaunchConfiguration("lio_base_odom_topic"),
                "source_child_frame": LaunchConfiguration("lio_sensor_frame"),
                "target_child_frame": LaunchConfiguration("base_frame"),
            }
        ],
    )

    local_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[
            LaunchConfiguration("local_config_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom0": LaunchConfiguration("lio_base_odom_topic"),
                "publish_tf": LaunchConfiguration("publish_local_tf"),
            },
        ],
        remappings=[
            ("odometry/filtered", LaunchConfiguration("local_odom_topic")),
        ],
    )

    global_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="screen",
        parameters=[
            LaunchConfiguration("global_config_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom0": LaunchConfiguration("local_odom_topic"),
                "odom1": LaunchConfiguration("smoothed_gps_odom_topic"),
                "publish_tf": LaunchConfiguration("publish_global_tf"),
            },
        ],
        remappings=[
            ("odometry/filtered", LaunchConfiguration("global_odom_topic")),
        ],
    )

    gps_recovery_smoother = Node(
        package="robot_ekf_localization",
        executable="gps_recovery_smoother_node",
        name="gps_recovery_smoother",
        output="screen",
        parameters=[
            LaunchConfiguration("global_config_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "raw_gps_topic": LaunchConfiguration("gps_odom_topic"),
                "local_odom_topic": LaunchConfiguration("local_odom_topic"),
                "output_topic": LaunchConfiguration(
                    "smoothed_gps_odom_topic"
                ),
            },
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
                "local_config_file",
                default_value=default_local_config,
                description="局部EKF参数文件",
            ),
            DeclareLaunchArgument(
                "global_config_file",
                default_value=default_global_config,
                description="全局EKF参数文件",
            ),
            DeclareLaunchArgument(
                "lio_sensor_odom_topic",
                default_value="/Odometry",
                description="FAST-LIO原生IMU参考点里程计",
            ),
            DeclareLaunchArgument(
                "lio_base_odom_topic",
                default_value="/odometry/lio/base",
                description="经URDF/TF转换到融合参考点的激光里程计",
            ),
            DeclareLaunchArgument(
                "lio_sensor_frame",
                default_value="livox_imu",
                description="FAST-LIO原生位姿参考frame",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="所有里程计统一转换到的融合参考frame",
            ),
            DeclareLaunchArgument(
                "publish_robot_description",
                default_value="true",
                description="是否由本launch发布URDF静态TF",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value=os.path.join(
                    description_share, "urdf", "robot.urdf"
                ),
                description="传感器安装外参的唯一URDF文件",
            ),
            DeclareLaunchArgument(
                "gps_odom_topic",
                default_value="/odometry/gps",
                description="转换到map坐标系的GPS绝对里程计输入",
            ),
            DeclareLaunchArgument(
                "smoothed_gps_odom_topic",
                default_value="/odometry/gps/smoothed",
                description="GPS消失/恢复状态机输出的绝对观测",
            ),
            DeclareLaunchArgument(
                "local_odom_topic",
                default_value="/odometry/local",
                description="局部EKF输出",
            ),
            DeclareLaunchArgument(
                "global_odom_topic",
                default_value="/odometry/global",
                description="GPS修正后的全局融合里程计输出",
            ),
            DeclareLaunchArgument(
                "publish_local_tf",
                default_value="false",
                description="固定map=odom模式下关闭局部EKF的重复odom->base_footprint",
            ),
            DeclareLaunchArgument(
                "publish_global_tf",
                default_value="true",
                description="由全局EKF发布GPS修正后的odom->base_footprint",
            ),
            robot_description,
            static_map_to_odom,
            lio_reference_transform,
            local_ekf,
            gps_recovery_smoother,
            global_ekf,
        ]
    )
