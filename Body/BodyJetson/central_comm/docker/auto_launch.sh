#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

cd /home/ros/ros2_ws

mkdir -p build install log

# Forced clean rebuild
if [ -n "${FORCE_REBUILD:-}" ]; then
  echo "[auto_launch] FORCE_REBUILD set -> cleaning build/install/log contents"

  rm -rf build/* build/.[!.]* build/..?* 2>/dev/null || true
  rm -rf install/* install/.[!.]* install/..?* 2>/dev/null || true
  rm -rf log/* log/.[!.]* log/..?* 2>/dev/null || true
fi

# Always perform an incremental build.
# Existing build/install directories are reused, so unchanged packages stay fast.
echo "[auto_launch] Building workspace..."

colcon build \
  --merge-install \
  --symlink-install \
  --packages-select \
    tcp_msg \
    serial_msg \
    xiao_bridge \
    body_mpu_reader \
    motor_control \
    drive_control

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
    set +e
    # run the node; if it fails it exits and retries in 2s
    stdbuf -oL -eL ros2 run xiao_bridge bridge_node \
      --ros-args -p ip:=${ip} -p port:=${port} -r __ns:=${ns} \
      >> "${logfile}" 2>&1

    rc=$?
    set -e
    echo "[bridge:${name}] $(date +'%F %T') exited (rc=${rc}); retrying in 2s…" | tee -a "${logfile}"
    sleep 2
  done
}

run_body_imu() {
  local bus="${1:-7}"
  local addr="${2:-0x68}"
  local rate="${3:-50.0}"
  local topic="${4:-Body/mpu}"
  local logfile="${ROS_LOG_DIR}/body_mpu.log"

  echo "[body_mpu] starting loop -> i2c_bus=${bus} i2c_addr=${addr} rate=${rate}Hz topic=${topic} (log: ${logfile})"

  while true; do
    echo "[body_mpu] $(date +'%F %T') starting process…"
    set +e
    stdbuf -oL -eL ros2 run body_mpu_reader body_mpu_node \
      --ros-args -p i2c_bus:=${bus} -p i2c_address:=${addr} -p publish_rate:=${rate} -p topic_name:=${topic} \
      >> "${logfile}" 2>&1

    rc=$?
    set -e
    echo "[body_mpu] $(date +'%F %T') exited (rc=${rc}); retrying in 2s…" | tee -a "${logfile}"
    sleep 2
  done
}

run_motor_control() {
  local logfile="${ROS_LOG_DIR}/motor_control.log"

  echo "[motor_control] starting loop (log: ${logfile})"

  while true; do
    echo "[motor_control] $(date +'%F %T') starting process…"
    set +e
    stdbuf -oL -eL ros2 run motor_control motor_control \
      >> "${logfile}" 2>&1

    rc=$?
    set -e
    echo "[motor_control] $(date +'%F %T') exited (rc=${rc}); retrying in 2s…" | tee -a "${logfile}"
    sleep 2
  done
}

run_drive_control() {
  local logfile="${ROS_LOG_DIR}/drive_control.log"

  echo "[drive_control] starting loop (log: ${logfile})"

  while true; do
    echo "[drive_control] $(date +'%F %T') starting process…"
    set +e
    stdbuf -oL -eL ros2 launch drive_control launch.py \
      >> "${logfile}" 2>&1

    rc=$?
    set -e
    echo "[drive_control] $(date +'%F %T') exited (rc=${rc}); retrying in 2s…" | tee -a "${logfile}"
    sleep 2
  done
}

# Trap signals and forward to children
pids=()
trap 'echo "[auto_launch] signal received, stopping…"; kill "${pids[@]}" 2>/dev/null || true; wait; exit 0' INT TERM

# Launch Body IMU first
run_body_imu 7 0x68 50.0 "/Body/mpu" &
pids+=($!)

# Launches both bridges (independent retries)
run_bridge left  "$ESP_L" "$PORT_L" "/leg_l" &
pids+=($!)

run_bridge right "$ESP_R" "$PORT_R" "/leg_r" &
pids+=($!)

run_motor_control &
pids+=($!)

run_drive_control &
pids+=($!)

# Keep PID 1 alive
wait -n || true
 # If one dies, it still waits
 wait
