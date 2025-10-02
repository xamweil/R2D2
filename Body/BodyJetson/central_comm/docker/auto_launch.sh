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
  echo "[auto_launch] Building xiao_bridge (symlink install)…"
  colcon build --symlink-install --packages-up-to xiao_bridge
else
  echo "[auto_launch] Using existing build/install."
fi

source /home/ros/ros2_ws/install/setup.bash

ESP_L=192.168.66.10
ESP_R=192.168.66.11
PORT_L=5010
PORT_R=5011



# Helper for node launch
run_bridge() {
  local name="$1" ip="$2" port="$3" ns="$4"
  local logfile="${ROS_LOG_DIR}/bridge_${name}.log"

  echo "[bridge:${name}] starting loop -> ${ip}:${port} ns=${ns} (log: ${logfile})"

  # line-buffer stdout/stderr so logs stream
  while true; do
    echo "[bridge:${name}] $(date +'%F %T') starting process…"
    # run the node; if it fails it exits and retries in 2s
    stdbuf -oL -eL ros2 run xiao_bridge bridge_node \
      --ros-args -p ip:=${ip} -p port:=${port} -r __ns:=${ns} \
      >> "${logfile}" 2>&1 || true

    rc=$?
    echo "[bridge:${name}] $(date +'%F %T') exited (rc=${rc}); retrying in 2s…" | tee -a "${logfile}"
    sleep 2
  done
}

# Trap signals and forward to children
pids=()
trap 'echo "[auto_launch] signal received, stopping…"; kill "${pids[@]}" 2>/dev/null || true; wait; exit 0' INT TERM

# Launches both bridges (independent retries)
run_bridge left  "$ESP_L" "$PORT_L" "/leg_l" &
pids+=($!)

run_bridge right "$ESP_R" "$PORT_R" "/leg_r" &
pids+=($!)

# Keep PID 1 alive
wait -n || true
 # If one dies, it still waits
 wait
