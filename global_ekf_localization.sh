#!/bin/bash

# This script is used to launch the Fast LIO mapping node.

source install/setup.bash

ros2 launch robot_ekf_localization global_ekf_localization.launch.py