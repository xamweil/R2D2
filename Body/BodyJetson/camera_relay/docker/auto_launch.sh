#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

cd /home/ros/ros2_ws

# Forced rebuild
if [ -n "${FORCE_REBUILD:-}" ]; then
  echo "[auto_launch] FORCE_REBUILD set -> cleaning build/install/log"
  rm -rf build install log
fi

# Build only if needed
if [ ! -f "install/setup.bash" ] || [ -z "$(ls -A build 2>/dev/null)" ]; then
  echo "[auto_launch] Building workspace (merged install, symlink)…"
  source /opt/ros/humble/setup.bash
  colcon build --merge-install --symlink-install --packages-select camera_relay
else
  echo "[auto_launch] Using existing build/install."
fi

source /home/ros/ros2_ws/install/setup.bash

LOGFILE="${ROS_LOG_DIR}/camera_relay.launch.log"

echo "[auto_launch] starting camera_relay launch loop (log: ${LOGFILE})"

run_launch() {
  while true; do
    echo "[auto_launch] $(date +'%F %T') starting launch…" | tee -a "${LOGFILE}"

    stdbuf -oL -eL ros2 launch camera_relay camera_relay.launch.py \
      input_compressed_topic:="${INPUT_COMPRESSED_TOPIC:-/camera/image_raw/compressed}" \
      input_camera_info_topic:="${INPUT_CAMERA_INFO_TOPIC:-/camera/camera_info}" \
      output_compressed_topic:="${OUTPUT_COMPRESSED_TOPIC:-/relay/camera/image_raw/compressed}" \
      output_raw_topic:="${OUTPUT_RAW_TOPIC:-/relay/camera/image_raw}" \
      output_camera_info_topic:="${OUTPUT_CAMERA_INFO_TOPIC:-/relay/camera/camera_info}" \
      >> "${LOGFILE}" 2>&1 || true

    rc=$?
    echo "[auto_launch] $(date +'%F %T') launch exited (rc=${rc}); retrying in 2s…" | tee -a "${LOGFILE}"
    sleep 2
  done
}

trap 'echo "[auto_launch] signal received, stopping…"; exit 0' INT TERM

run_launch