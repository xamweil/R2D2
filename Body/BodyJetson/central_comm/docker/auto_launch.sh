#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash

cd /home/ros/ros2_ws

# Forced rebuild
if [ -n "${FORCE_REBUILD:-}" ]; then
  echo "[auto_launch] FORCE_REBUILD set -> cleaning build/install/log"
  rm -rf build install log
fi

# Build only if needed (first run / clean tree)
if [ ! -f "install/setup.bash" ] || [ -z "$(ls -A build 2>/dev/null)" ]; then
  echo "[auto_launch] Building workspace (merged install, symlink)…"
  colcon build --merge-install \
    --packages-select tcp_msg serial_msg body_imu_bus motor_control
else
  echo "[auto_launch] Using existing build/install."
fi

source /home/ros/ros2_ws/install/setup.bash

# Helper for node launch
run_body_imu_bus() {
  local bus="${1:-7}"
  local mux_addr="${2:-0x70}"
  local rate="${3:-50}"
  local retry="${4:-10.0}"
  local logfile="${ROS_LOG_DIR}/body_imu_bus.log"

  echo "[body_imu_bus] starting loop -> i2c_bus=${bus} mux_addr=${mux_addr} rate=${rate}Hz retry=${retry}s (log: ${logfile})"

  while true; do
    echo "[body_imu_bus] $(date +'%F %T') starting process…"
    stdbuf -oL -eL ros2 run body_imu_bus body_imu_bus_node \
      --ros-args \
      -p i2c_bus:=${bus} \
      -p mux_address:=${mux_addr} \
      -p poll_rate_hz:=${rate} \
      -p init_retry_interval_sec:=${retry} \
      >> "${logfile}" 2>&1 || true

    rc=$?
    echo "[body_imu_bus] $(date +'%F %T') exited (rc=${rc}); retrying in 2s…" | tee -a "${logfile}"
    sleep 2
  done
}

run_motor_control() {
  local logfile="${ROS_LOG_DIR}/motor_control.log"

  echo "[motor_control] starting loop (log: ${logfile})"

  while true; do
    echo "[motor_control] $(date +'%F %T') starting process…"
    stdbuf -oL -eL ros2 run motor_control motor_control \
      >> "${logfile}" 2>&1 || true

    rc=$?
    echo "[motor_control] $(date +'%F %T') exited (rc=${rc}); retrying in 2s…" | tee -a "${logfile}"
    sleep 2
  done
}

# Trap signals and forward to children
pids=()
trap 'echo "[auto_launch] signal received, stopping…"; kill "${pids[@]}" 2>/dev/null || true; wait; exit 0' INT TERM

# Launch Body IMU bus first
run_body_imu_bus 7 0x70 50 10.0 &
pids+=($!)

run_motor_control &
pids+=($!)

# Keep PID 1 alive
wait -n || true
wait