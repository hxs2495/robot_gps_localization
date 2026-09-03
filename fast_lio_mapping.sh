#!/bin/bash

# This script is used to launch the Fast LIO mapping node.
set -e

source install/setup.bash

ros2 launch fast_lio mapping.launch.py "$@"
