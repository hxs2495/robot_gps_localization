#!/bin/bash

set -e

source install/setup.bash
ros2 launch robot_fusion_path fusion_path.launch.py "$@"
