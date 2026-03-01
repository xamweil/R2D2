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

# Camera params (override via docker-compose environment)
: "${CAM_WIDTH:=1280}"
: "${CAM_HEIGHT:=720}"
: "${CAM_FPS:=30}"
: "${CAM_FRAME_ID:=camera_optical_frame}"
: "${CAM_NODE_NAME:=camera}"

CAM_FRAME_US=$((1000000 / CAM_FPS))
echo "[auto_launch] Launching camera node… (${CAM_WIDTH}x${CAM_HEIGHT}@${CAM_FPS})"

ros2 run camera_ros camera_node --ros-args \
  -r __node:=${CAM_NODE_NAME} \
  -p width:=${CAM_WIDTH} \
  -p height:=${CAM_HEIGHT} \
  -p FrameDurationLimits:="[${CAM_FRAME_US},${CAM_FRAME_US}]" \
  -p frame_id:=${CAM_FRAME_ID} \
  & PID1=$!

trap 'kill $PID1 2>/dev/null || true; exit 0' INT TERM
wait -n "$PID1"