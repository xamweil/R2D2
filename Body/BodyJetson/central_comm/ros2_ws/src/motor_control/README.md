# motor_control

ROS2 node that receives motor commands and serializes them into a binary protocol frame sent to the Raspberry Pi Pico over serial (`/dev/ttyACM0`).

## Subscribed topics

| Topic             | Type                      | Description              |
|------------------|---------------------------|--------------------------|
| `/motor_command` | `serial_msg/msg/MotorCommand` | Motor command messages |
| `/Head/mpu`      | `tcp_msg/msg/MPU6500Sample`   | Head IMU sample        |
| `/Body/mpu`      | `tcp_msg/msg/MPU6500Sample`   | Body IMU sample        |
| `/leg_l/imu/foot`| `tcp_msg/msg/MPU6500Sample`   | Left foot IMU sample   |
| `/leg_l/imu/leg` | `tcp_msg/msg/MPU6500Sample`   | Left leg IMU sample    |
| `/leg_r/imu/foot`| `tcp_msg/msg/MPU6500Sample`   | Right foot IMU sample  |
| `/leg_r/imu/leg` | `tcp_msg/msg/MPU6500Sample`   | Right leg IMU sample   |

## Message format

`MotorCommand` uses bounded dynamic arrays (up to 6 entries). Each index corresponds to one motor command — all arrays must be the same length.

| Field        | Type          | Description                        |
|--------------|---------------|------------------------------------|
| `ids`        | `uint8[<=6]`  | Motor indices to update            |
| `enable`     | `bool[<=6]`   | Enable motor                       |
| `direction`  | `bool[<=6]`   | Direction (`false`=forward)        |
| `angle_set`  | `bool[<=6]`   | Apply `angle` value                |
| `velocity_set` | `bool[<=6]` | Apply `velocity` value             |
| `angle`      | `float32[<=6]`| Target angle in degrees            |
| `velocity`   | `uint8[<=6]`  | Target velocity (0–100)            |

### Motor indices

| Index | Name         |
|-------|--------------|
| 0     | mid_foot     |
| 1     | head         |
| 2     | left_shoulder|
| 3     | right_shoulder|

## Running

```bash
ros2 launch motor_control launch.py
```
The node sends motor commands on change and forwards cached IMU data periodically to the Pico.

## Sending commands

Set a single motor via CLI:
```bash
ros2 topic pub --once /motor_command serial_msg/msg/MotorCommand \
  "{ids: [1], enable: [true], direction: [false], angle_set: [true], velocity_set: [false], angle: [45.0], velocity: [0]}"
```

Set multiple motors in one message:
```bash
ros2 topic pub --once /motor_command serial_msg/msg/MotorCommand \
  "{ids: [1, 2], enable: [true, true], direction: [false, true], angle_set: [true, false], velocity_set: [false, true], angle: [45.0, 0.0], velocity: [0, 128]}"
```

## Wire protocol

Two frame types are sent over serial:

### 1) Motor command frame

Sent immediately on command updates, with a minimum spacing of 50 ms.
```
byte 0: 0xAA (start-of-frame)
bytes 1–4: control word (u32 LE) — 4 bits per motor: enable | direction | angle_set | velocity_set
bytes 5–34: 5 bytes per motor × 6 motors — float32 LE angle + uint8 velocity
```

### 2) IMU frame

Sent periodically at ~40 Hz.

Order of IMUs:
1. head
2. body
3. left foot
4. left leg
5. right foot
6. right leg

Each IMU contributes:

- `accel_x` (`int16`)
- `accel_y` (`int16`)
- `accel_z` (`int16`)
- `ts_ms` (`uint32`)

```
byte 0: 0xAB (start-of-frame)
bytes 1–60: 6 IMUs × 10 bytes each
```

If an IMU is missing or stale, its forwarded `ts_ms` is set to `0`.
