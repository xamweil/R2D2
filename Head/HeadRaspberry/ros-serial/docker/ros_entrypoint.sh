#!/usr/bin/env bash
set -e
# Source ROS + your overlay if present
source /opt/ros/humble/setup.bash
[ -f /home/ros/ros2_ws/install/setup.bash ] && source /home/ros/ros2_ws/install/setup.bash
exec "$@"