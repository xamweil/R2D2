# motor_control

ROS2 node that receives motor commands and serializes them into a binary protocol frame sent to the Raspberry Pi Pico over serial (`/dev/ttyACM0`).

## Subscribed topics

| Topic           | Type                            | Description            |
|-----------------|---------------------------------|------------------------|
| `/motor_command` | `serial_msg/msg/MotorCommand` | Motor command messages |

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
| `velocity`   | `uint8[<=6]`  | Target velocity (0–255)            |

### Motor indices

| Index | Name         |
|-------|--------------|
| 0     | mid_foot     |
| 1     | head         |
| 2     | left_shoulder|
| 3     | right_shoulder|
| 4     | left_foot    |
| 5     | right_foot   |

## Running

```bash
ros2 launch motor_control launch.py
```

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

Frames are sent over serial at 50 ms intervals (timer) or immediately on receipt of a command. Each frame is 35 bytes:

```
byte 0:     0xAA  (start-of-frame)
bytes 1–4:  control word (u32 LE) — 4 bits per motor: enable | direction | angle_set | velocity_set
bytes 5–34: 5 bytes per motor × 6 motors — float32 LE angle + uint8 velocity
```
