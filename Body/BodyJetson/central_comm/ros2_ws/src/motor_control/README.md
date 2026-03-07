# motor_control (ROS2 Node)

ROS2 node that receives motor commands and forwards them to the Pico over serial (`/dev/ttyACM0` at 115200 baud). Sends a 31-byte binary frame at 20 Hz.

## Build

Inside the Docker container (or a ROS2 Humble environment):

```bash
cd central_comm/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select serial_msg motor_control
source install/setup.bash
```

Or build via Docker:

```bash
cd Body/BodyJetson
docker compose up -d --build
```

## Run

```bash
ros2 launch motor_control launch.py
```

Or directly:

```bash
ros2 run motor_control motor_control
```

## Motor Command Interface

Topic: `/motor_command`
Message type: `serial_msg/msg/MotorCommand`

| Field       | Type   | Description              |
|-------------|--------|--------------------------|
| `id`        | uint8  | Motor index (0-5)        |
| `enable`    | bool   | Enable the motor         |
| `direction` | bool   | Rotation direction       |
| `frequency` | uint32 | PWM frequency in Hz      |

### Motor ID Mapping

| ID | Motor           |
|----|-----------------|
| 0  | mid_foot        |
| 1  | head            |
| 2  | left_shoulder   |
| 3  | right_shoulder  |
| 4  | left_foot       |
| 5  | right_foot      |

### Example

```bash
ros2 topic pub --once /motor_command serial_msg/msg/MotorCommand "{id: 1, enable: false, direction: false, frequency: 0}"
```
