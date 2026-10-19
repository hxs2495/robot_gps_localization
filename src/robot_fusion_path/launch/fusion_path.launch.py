#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fusion_path_node = Node(
        package="robot_fusion_path",
        executable="fusion_path_node",
        name="fusion_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_paths")),
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

    global_localization_node = Node(
        package="robot_fusion_path",
        executable="global_localization_node",
        name="global_localization_node",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("publish_global_localization")
        ),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "gps_topic": LaunchConfiguration("gps_topic"),
                "orientation_topic": LaunchConfiguration(
                    "orientation_topic"
                ),
                "odometry_topic": LaunchConfiguration("odom_topic"),
                "output_topic": LaunchConfiguration(
                    "global_localization_topic"
                ),
                "output_fix_topic": LaunchConfiguration(
                    "global_fix_topic"
                ),
                "base_frame_id": LaunchConfiguration("base_frame_id"),
                "gps_frame_id": LaunchConfiguration("gps_frame_id"),
                "use_orientation": LaunchConfiguration("use_orientation"),
                "orientation_convention": LaunchConfiguration(
                    "orientation_convention"
                ),
                "orientation_yaw_offset": LaunchConfiguration(
                    "orientation_yaw_offset"
                ),
                "initial_enu_yaw": LaunchConfiguration(
                    "initial_enu_yaw"
                ),
                "two_d_mode": LaunchConfiguration("two_d_mode"),
                "publish_nav_sat_fix": LaunchConfiguration(
                    "publish_nav_sat_fix"
                ),
                "max_anchor_time_offset": LaunchConfiguration(
                    "max_anchor_time_offset"
                ),
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
                "publish_paths",
                default_value="true",
                description="是否发布local/global Path",
            ),
            DeclareLaunchArgument(
                "publish_global_localization",
                default_value="true",
                description="是否发布WGS84全球融合定位",
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
            DeclareLaunchArgument(
                "gps_topic",
                default_value="/fix/filter",
                description="用于建立全球原点的有效NavSatFix话题",
            ),
            DeclareLaunchArgument(
                "orientation_topic",
                default_value="/imu_orientation",
                description="用于建立全球姿态的GNSS/双天线航向话题",
            ),
            DeclareLaunchArgument(
                "global_localization_topic",
                default_value="/localization/global",
                description="完整全球融合定位自定义消息话题",
            ),
            DeclareLaunchArgument(
                "global_fix_topic",
                default_value="/localization/global_fix",
                description="兼容NavSatFix的全球融合位置话题",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value="base_footprint",
                description="融合定位描述的车体参考frame",
            ),
            DeclareLaunchArgument(
                "gps_frame_id",
                default_value="gps",
                description="NavSatFix对应的天线frame，用于查询URDF杆臂",
            ),
            DeclareLaunchArgument(
                "orientation_convention",
                default_value="north_clockwise",
                description="航向约定：north_clockwise或enu",
            ),
            DeclareLaunchArgument(
                "use_orientation",
                default_value="true",
                description="是否用航向话题初始化全球姿态",
            ),
            DeclareLaunchArgument(
                "orientation_yaw_offset",
                default_value="0.0297",
                description="航向固定零偏(rad)，需与GPS转UTM节点保持一致",
            ),
            DeclareLaunchArgument(
                "initial_enu_yaw",
                default_value="0.0",
                description="不使用航向话题时的初始ENU yaw(rad)",
            ),
            DeclareLaunchArgument(
                "two_d_mode",
                default_value="true",
                description="是否将输出姿态限制为平面yaw",
            ),
            DeclareLaunchArgument(
                "publish_nav_sat_fix",
                default_value="true",
                description="是否额外发布NavSatFix兼容输出",
            ),
            DeclareLaunchArgument(
                "max_anchor_time_offset",
                default_value="0.2",
                description="初始化时GPS/航向与融合里程计最大时间差(s)",
            ),
            fusion_path_node,
            global_localization_node,
        ]
    )
