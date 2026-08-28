from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,TimerAction
from launch.substitutions import LaunchConfiguration
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1. GPS滤波
    gpgga_filter_node = Node(
        package='robot_odom_transform',
        executable='gpgga_filter_node',
        name='gpgga_filter_node',
        output='screen',
        parameters=[{
            "min_sats": 15,
            "required_status": -1,
            "recover_frame_count": 10,
        }],
        remappings=[
            ('/fix', '/fix'),
            ('/fix/filter', '/fix/filter'),
        ],
    )
    gps_to_utm_odometry_node = Node(
        package='robot_odom_transform',
        executable='gps_to_utm_odometry_node',
        name='gps_to_utm_odometry_node',
        output='screen',
        parameters=[{
                "gps_topic": "/fix/filter",  # GPS话题
                "odom_topic": "utm/gps",  # 输出的UTM里程计话题
                "zone_northp_topic": "utm/zone/northp",  # UTM分区和北半球标志话题
                "frame_id": "utm",  # 输出的UTM里程计坐标系
                "child_frame_id": "gps",  # 输出的UTM里程计子坐标系
                "use_velocity": False,  # 是否进行航向推算
                "use_yaw": False,  # 是否进行速度推算
        }],
    )

    utm_to_gps_odometry_node = Node(
        package='robot_odom_transform',
        executable='utm_to_gps_odometry_node',
        name='utm_to_gps_odometry_node',
        output='screen',
        parameters=[{
                "input_odom_topic": "utm/gps",  
                "zeroed_odom_topic": "odometry/gps",  
                "initial_pose_topic": "utm/initial_pose",
        }],
    )
    
    robot_calibration_data_recode = Node(
        package='robot_calibration_manager',
        executable='robot_calibration_data_recode',
        name='robot_calibration_data_recode',
        output='screen',
    )
    
    fast_lio = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('fast_lio'),
                    '/launch',
                    '/mapping.launch.py'
                ]),
            )
        ]
    )

    # 返回启动描述
    return LaunchDescription([
        gpgga_filter_node,
        gps_to_utm_odometry_node,
        utm_to_gps_odometry_node,
        robot_calibration_data_recode,
        fast_lio
        # ex_save_node
        # test_odom_transformstamp_node
    ])
