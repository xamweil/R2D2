#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash

cd /home/ros/ros2_ws

# Optional forced rebuild
if [ -n "${FORCE_REBUILD:-}" ]; then
  echo "[visual_perception:auto_launch] FORCE_REBUILD set -> cleaning build/install"
  rm -rf build install
fi

# Build only if there are ROS packages present
if [ -d "src" ] && [ -n "$(find src -mindepth 1 -maxdepth 1 2>/dev/null)" ]; then
  if [ ! -f "install/setup.bash" ] || [ -z "$(ls -A build 2>/dev/null)" ]; then
    echo "[visual_perception:auto_launch] Building workspace (symlink install)…"
    colcon build --symlink-install
  else
    echo "[visual_perception:auto_launch] Using existing build/install."
  fi

  [ -f "/home/ros/ros2_ws/install/setup.bash" ] && source /home/ros/ros2_ws/install/setup.bash
else
  echo "[visual_perception:auto_launch] No ROS packages in /home/ros/ros2_ws/src yet."
fi

echo "[visual_perception:auto_launch] Fetching required models if missing..."
python3 /home/ros/models/fetch_models.py

ENGINE_PATH="/home/ros/models/yolov8n/yolov8n.engine"

if [ ! -f "${ENGINE_PATH}" ]; then
  echo "[visual_perception:auto_launch] Engine missing -> building ${ENGINE_PATH}"
  python3 /home/ros/models/prepare_models.py --build yolov8n
else
  echo "[visual_perception:auto_launch] Engine already present: ${ENGINE_PATH}"
fi

echo "[visual_perception:auto_launch] Launching scene_understanding..."
exec ros2 launch scene_understanding scene_understanding.launch.py