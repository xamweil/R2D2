#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash

cd /home/ros/ros2_ws

# --- Sanity checks for mounted interface packages ---
for pkg in serial_msg; do
  if [ ! -f "/home/ros/ros2_ws/src/${pkg}/package.xml" ]; then
    echo "[action_layer:auto_launch] ERROR: expected /home/ros/ros2_ws/src/${pkg}/package.xml"
    echo "[action_layer:auto_launch] Hint: check docker-compose volume mounts for ${pkg}."
    exit 1
  fi
done

# Forced rebuild
if [ -n "${FORCE_REBUILD:-}" ]; then
  echo "[action_layer:auto_launch] FORCE_REBUILD set -> cleaning build/install"
  rm -rf build install
fi

# Build only if needed
if [ ! -f "install/setup.bash" ] || [ -z "$(ls -A build 2>/dev/null)" ]; then
  echo "[action_layer:auto_launch] Building workspace (symlink install)…"
  colcon build --symlink-install
else
  echo "[action_layer:auto_launch] Using existing build/install."
fi

source /home/ros/ros2_ws/install/setup.bash

echo "[action_layer:auto_launch] Launching behavior manager…"
exec ros2 launch behavior_manager behavior_manager.launch.py