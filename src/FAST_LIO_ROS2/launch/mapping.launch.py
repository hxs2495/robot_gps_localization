import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    description_path = get_package_share_directory('robot_description')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            description_path, 'launch', 'robot_description.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'urdf_file': LaunchConfiguration('urdf_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('publish_robot_description')),
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use rosbag /clock by default; set false for live sensors'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_publish_robot_description_cmd = DeclareLaunchArgument(
        'publish_robot_description', default_value='true',
        description='Publish the URDF static sensor TFs for standalone use'
    )
    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(description_path, 'urdf', 'robot.urdf'),
        description='Canonical URDF containing the LiDAR-IMU extrinsic'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_publish_robot_description_cmd)
    ld.add_action(declare_urdf_file_cmd)

    ld.add_action(robot_description)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
