# drive_control

ROS2 package for controlling DDSM115 drive motors over RS485.

The node runs inside the `central_comm` container and communicates with the USB-to-RS485 adapter exposed inside Docker as:

```text
/dev/ddsm_rs485
```

Baud rate:

```text
115200
```

## Structure

```text
include/drive_control/
├── DDSM115.hpp
├── DriveControlNode.hpp
├── DriveMotor.hpp
└── RS485Com.hpp

src/
├── DDSM115.cpp
├── DriveControlNode.cpp
├── DriveControlNodeMain.cpp
├── DriveMotor.cpp
└── RS485Com.cpp
```

### DDSM115

Low-level DDSM115 protocol implementation.

Responsible for:

* building DDSM115 command frames
* velocity, position, current and brake commands
* mode switching
* CRC
* decoding normal motor feedback
* storing latest motor state

This class is ROS-independent.

### RS485Com

Owns the serial connection to the USB-to-RS485 adapter.

Responsible for:

* opening/configuring `/dev/ddsm_rs485`
* registering DDSM115 motors
* round-robin communication
* sending mode changes before drive commands
* waiting for motor responses
* response timeout handling
* dispatching feedback to the correct motor

Only one normal DDSM115 request is outstanding at a time.

### DriveMotor

Robot-specific wrapper around one physical DDSM115 motor.

Contains:

* ROS command ID
* physical RS485 motor ID
* human-readable name
* motor group
* inversion setting

Motor configuration is stored in:

```cpp
DRIVE_MOTOR_CONFIGS
```

inside:

```text
include/drive_control/DriveMotor.hpp
```

### DriveControlNode

ROS2 interface for the drive motors.

Responsible for:

* subscribing to `/motor_command`
* mapping ROS commands to `DriveMotor`
* running RS485 communication
* automatically creating feedback publishers
* publishing latest feedback for every configured motor

### DriveControlNodeMain

Only starts, spins and shuts down `DriveControlNode`.

No motor-control logic should be added here.

---

## Adding a new motor

Add one entry to `DRIVE_MOTOR_CONFIGS` in:

```text
include/drive_control/DriveMotor.hpp
```

Example:

```cpp
{11, 11, "left_foot_back", MotorGroup::LEFT_FOOT, false},
```

Format:

```text
ROS command ID
RS485 motor ID
name
group
inverted
```

Example:

```cpp
static constexpr DriveMotorConfig DRIVE_MOTOR_CONFIGS[] =
{
    {11, 11, "left_foot_back",  MotorGroup::LEFT_FOOT,  false},
    {13, 13, "left_foot_front", MotorGroup::LEFT_FOOT,  false},
    {15, 15, "right_foot_back", MotorGroup::RIGHT_FOOT, false},
    {17, 17, "right_foot_front",MotorGroup::RIGHT_FOOT, false},
};
```

The node automatically:

* creates the `DriveMotor`
* registers it with `RS485Com`
* creates its ROS feedback publisher

No changes to `DriveControlNode.cpp` should be required.

Physical RS485 motor IDs must be unique.

---

## Motor command mapping

`serial_msg/msg/MotorCommand` is interpreted as:

```text
enable=false
    -> brake on

enable=true
angle_set=false
velocity_set=false
    -> brake off

velocity_set=true
angle_set=false
    -> velocity command

angle_set=true
velocity_set=false
    -> position command

angle_set=true
velocity_set=true
    -> reserved for special DDSM115 feedback request
```

`direction` is currently used for velocity direction.

---

## Feedback

Each configured motor gets its own topic:

```text
/ddsm115_feedback/<motor_name>_<command_id>
```

Examples:

```text
/ddsm115_feedback/left_foot_back_11
/ddsm115_feedback/left_foot_front_13
/ddsm115_feedback/right_foot_back_15
/ddsm115_feedback/right_foot_front_17
```

Message type:

```text
serial_msg/msg/DDSM115Feedback
```

Current fields:

```text
uint8 id
uint8 mode
int16 velocity
float32 current
float32 position
uint8 error_code
```

Units:

```text
velocity -> RPM
current  -> A
position -> degrees
```

`error_code` is published as the raw DDSM115 error byte and can be decoded elsewhere.

---

## Launch

```bash
ros2 launch drive_control launch.py
```

The node is also started automatically by the `central_comm` `auto_launch.sh`.

---

## TODO

* Implement special DDSM115 `0x74` feedback request.
* Map `angle_set=true` + `velocity_set=true` to the `0x74` request.
* Define handling/parsing for the additional `0x74` feedback frame.
* Add feedback-validity information so ROS consumers can distinguish real zero values from "no feedback received yet".
* Define position inversion/mirroring behavior for inverted motors.
* Add safe handling for entering position mode only when motor speed is below the DDSM115-required threshold.
* Add human-readable decoding/documentation for DDSM115 error-code bits.
