#!/bin/bash

# Unified startup: publish the URDF once, then start FAST-LIO, GPS conversion,
# reference-point conversion, and both EKFs.
set -e

source install/setup.bash

ros2 launch robot_ekf_localization localization_bringup.launch.py "$@"
