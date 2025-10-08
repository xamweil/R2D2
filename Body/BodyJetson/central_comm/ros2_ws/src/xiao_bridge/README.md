

---

# Xiao Bridge (Jetson / ROS 2)

ROS 2 packages that connect to one ESP32-C3 (XIAO) over TCP, republish two MPU-6500 IMU streams, log “taster” button presses, and expose simple motor commands as a service.

* **Package**: `xiao_bridge` — the TCP client and ROS 2 node.&#x20;
* **Package**: `tcp_msg` — custom message/service definitions.
* **Test tools**: a live terminal dashboard (`Test.py`) and a simple sequence test (`XiaoESP32C3Test.py`).


## What it does

* Opens a TCP socket to the ESP (`ip`, `port` parameters). IMU samples arrive as 16-byte payloads: `ax, ay, az, gx, gy, gz, ts_ms(u32)` (little-endian). The node unpacks to `tcp_msg/MPU6500Sample` and publishes on `imu/leg` and `imu/foot`.&#x20;
* Logs “Taster 1 pressed” / “Taster 2 pressed” to `/rosout` when it receives unsolicited taster frames. The provided dashboard watches `/rosout` to show recent presses.
* Exposes a `cmd` service (`tcp_msg/XiaoCmd`) to send motor/utility commands; replies are parsed to human-readable status.&#x20;
* The TCP layer auto-reconnects if the ESP drops and delivers frames to the bridge via callbacks.&#x20;

---

## Build

From the workspace root (the directory that contains `central_comm/`):

```bash
# From your ROS 2 Humble environment
cd central_comm
colcon build --packages-select tcp_msg xiao_bridge
source install/setup.bash
```

---

## Run

### Single ESP (one leg)

```bash
# Example: left leg device at 192.168.66.10:5010 (namespace /leg_l)
ros2 run xiao_bridge bridge_node \
  --ros-args -p ip:=192.168.66.10 -p port:=5010 -r __ns:=/leg_l
```

* Parameters: `~ip` (default `192.168.66.10`), `~port` (default `5010`).&#x20;
* Publishers: `imu/leg`, `imu/foot` (`tcp_msg/MPU6500Sample`).&#x20;
* Service: `cmd` (`tcp_msg/XiaoCmd`).&#x20;

### Two ESPs (left + right)

```bash
# Left
ros2 run xiao_bridge bridge_node \
  --ros-args -p ip:=192.168.66.10 -p port:=5010 -r __ns:=/leg_l &
# Right
ros2 run xiao_bridge bridge_node \
  --ros-args -p ip:=192.168.66.11 -p port:=5011 -r __ns:=/leg_r &
```

> You can also remap topic names if needed (e.g., `--remap imu/leg:=leg_l/imu/leg`), but using namespaces is simpler.

---

## Topics & messages

**Published topics**

* `imu/leg`  (`tcp_msg/MPU6500Sample`)
* `imu/foot` (`tcp_msg/MPU6500Sample`)&#x20;

**`tcp_msg/MPU6500Sample` fields (as used by the bridge)**

* `int16[3] accel` — raw counts (scale to g with `1/16384` if FS=±2 g).
* `int16[3] gyro` — raw counts (scale to dps with `1/131` if FS=±250 dps).
* `uint32 ts_ms` — UTC milliseconds from the ESP (SNTP-synced).
  (See `Test.py` for example scaling.)&#x20;

**Service**

* `cmd` (`tcp_msg/XiaoCmd`): forwards a `(cid, fid, args[])` frame and returns a parsed `message` and `status` (`0` for OK).&#x20;

---

## How the transport works

* The `Transport` thread reads frames, dispatches **IMU samples** to the bridge via `set_sample_callback`, **taster presses** to `set_taster_callback`, and queues **command replies** for `send_frame()` to await. It automatically **reconnects** on socket errors.&#x20;
* IMU payloads are delivered as the raw 16 bytes; the bridge unpacks with `struct.unpack("<hhhhhhI", payload)` and publishes.&#x20;

---

## Test tools

### 1) Terminal dashboard (`Test.py`)

Live, 1 Hz terminal UI that shows per-leg sample rates, latest accel/gyro (scaled), last timestamp, and recent taster presses (watched via `/rosout`). Choose left/right/both.

```bash
# after starting the bridge nodes
python3 central_comm/src/xiao_bridge/xiao_bridge/Test.py --side both
# or: --side left   or   --side right
```

* Subscribes to `/{leg_ns}/imu/leg` and `/{leg_ns}/imu/foot`.
* Considers a taster “ACTIVE” if a press log was seen in the last 1.5 s.&#x20;

### 2) Simple sequence test (`XiaoESP32C3Test.py`)

Grabs one leg+foot sample, then calls the `cmd` service to lock/unlock, and prompts you to press both tasters.

```bash
# with one bridge running in the current environment
python3 central_comm/src/xiao_bridge/xiao_bridge/XiaoESP32C3Test.py
```

* Shows scaled IMU readings and the service result strings.&#x20;

---

## Troubleshooting

* **No samples?** Check the IP/port parameters; the transport logs reconnect attempts on disconnects.&#x20;
* **Bad payload length warnings**: the bridge expects exactly **16-byte** IMU payloads (6×`int16` + `uint32`).&#x20;
* **Taster not showing in dashboard**: presses are detected via `/rosout` messages (`"Taster 1 pressed"` / `"Taster 2 pressed"`). Ensure the bridge prints those logs (it does on FID `0x01/0x02`).

---

## Notes

* Use ROS 2 namespaces (e.g., `/leg_l`, `/leg_r`) when running multiple bridges. The dashboard defaults to `leg_l` and `leg_r` but allows overrides.&#x20;
* The transport interprets 1-byte status replies into human-readable strings (e.g., “OK”, “Unknown function”). Service returns those.&#x20;

---

