#!/bin/bash

# Standalone global EKF and GPS loss/recovery smoother.
set -e

source install/setup.bash

ros2 launch robot_ekf_localization global_ekf_localization.launch.py "$@"
