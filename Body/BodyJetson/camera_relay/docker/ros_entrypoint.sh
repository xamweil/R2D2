#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
[ -f /home/ros/ros2_ws/install/setup.bash ] && source /home/ros/ros2_ws/install/setup.bash

exec "$@"