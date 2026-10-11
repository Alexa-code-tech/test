#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"
colcon build --packages-select ab_nav
source install/setup.bash
ros2 launch ab_nav sim.launch.py "$@"
