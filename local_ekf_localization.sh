#!/bin/bash

# This script is used to launch the Fast LIO mapping node.

source install/setup.bash

ros2 launch robot_ekf_localization local_ekf_localization.launch.py