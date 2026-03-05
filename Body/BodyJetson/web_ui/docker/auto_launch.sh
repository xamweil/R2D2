#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash

cd /home/ros/ros2_ws

# --- Sanity checks for mounted interface packages ---
for pkg in tcp_msg serial_msg; do
  if [ ! -f "/home/ros/ros2_ws/src/${pkg}/package.xml" ]; then
    echo "[web_ui:auto_launch] ERROR: expected /home/ros/ros2_ws/src/${pkg}/package.xml"
    echo "[web_ui:auto_launch] Hint: check docker-compose volume mounts for ${pkg}."
    exit 1
  fi
done

# Forced rebuild
if [ -n "${FORCE_REBUILD:-}" ]; then
  echo "[web_ui:auto_launch] FORCE_REBUILD set -> cleaning build/install"
  rm -rf build install
fi

# Build only if needed (first run / clean tree)
if [ ! -f "install/setup.bash" ] || [ -z "$(ls -A build 2>/dev/null)" ]; then
  echo "[web_ui:auto_launch] Building workspace (symlink install)…"
  colcon build --symlink-install
else
  echo "[web_ui:auto_launch] Using existing build/install."
fi

source /home/ros/ros2_ws/install/setup.bash

# API host/port configurable
export UI_BRIDGE_HOST="${UI_BRIDGE_HOST:-0.0.0.0}"
export UI_BRIDGE_PORT="${UI_BRIDGE_PORT:-8000}"

echo "[web_ui:auto_launch] Launching API node on ${UI_BRIDGE_HOST}:${UI_BRIDGE_PORT}…"
exec ros2 run ui_bridge api_node