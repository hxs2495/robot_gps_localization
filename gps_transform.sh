#!/bin/bash

# Launch the GPS transform nodes for a robot using ROS 2.
set -e

source install/setup.bash
ros2 launch robot_odom_transform start_gps_transform.launch.py "$@"
