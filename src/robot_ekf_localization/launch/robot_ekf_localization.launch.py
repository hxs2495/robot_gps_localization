import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(
        package='robot_cartographer'
    ).find('robot_cartographer')

    ekf_config = os.path.join(
        pkg_share,
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        ekf_node
    ])