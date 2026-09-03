#!/bin/bash

# Replays the recorded data together with the localization stack in a private
# ROS domain, avoiding mixed /clock and sensor publishers from Gazebo or tools.
set -e

source install/setup.bash

ros2 launch robot_ekf_localization localization_replay.launch.py "$@"
