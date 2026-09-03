#!/bin/bash

# Standalone LiDAR reference-point conversion and local EKF.
set -e

source install/setup.bash

# 固定map=odom融合链路中，由全局EKF唯一发布odom->base_footprint。
ros2 launch robot_ekf_localization local_ekf_localization.launch.py \
  publish_tf:=false "$@"
