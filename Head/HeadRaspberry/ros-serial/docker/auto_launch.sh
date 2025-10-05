#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash

cd /home/ros/ros2_ws

# Forced rebuild
if [ -n "${FORCE_REBUILD:-}" ]; then
  echo "[auto_launch] FORCE_REBUILD set -> cleaning build/install"
  rm -rf build install
fi

# Build only if needed (first run / clean tree)
if [ ! -f "install/setup.bash" ] || [ -z "$(ls -A build 2>/dev/null)" ]; then
  echo "[auto_launch] Building nodes (symlink install)…"
  colcon build --symlink-install

else
  echo "[auto_launch] Using existing build/install."
fi

source /home/ros/ros2_ws/install/setup.bash

echo "[auto_launch] Launching serial nodes…"
ros2 run serial_comm serial_node &  PID1=$!
ros2 run arduino_flash flash_node & PID2=$!

trap 'kill $PID1 $PID2 2>/dev/null || true; exit 0' INT TERM
wait -n "$PID1" "$PID2"

