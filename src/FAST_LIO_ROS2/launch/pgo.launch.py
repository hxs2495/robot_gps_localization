from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),

        Node(
            package='fast_lio',
            executable='alaserPGO',
            name='alaserPGO',
            output='screen',
            parameters=[{
                'scan_line': 64,
                'minimum_range': 0.5,
                'mapping_line_resolution': 0.4,
                'mapping_plane_resolution': 0.8,
                'mapviz_filter_size': 0.05,
                'keyframe_meter_gap': 0.5,
                'sc_dist_thres': 0.3,
                'sc_max_radius': 80.0,
                'lidar_type': 'OS1-64',
                'save_directory': '/home/orangepi/robot_xj_project/robot_data/maps/data/',
            }],
            remappings=[
                ('/aft_mapped_to_init', '/Odometry'),
                ('/velodyne_cloud_registered_local', '/cloud_registered_body'),
                ('/cloud_for_scancontext', '/cloud_registered_lidar'),
            ]
        ),
    ])
