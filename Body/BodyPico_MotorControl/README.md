# BodyPico_MotorControl

RP2040 (Raspberry Pi Pico) firmware that drives 6 stepper motors via PWM. Receives binary motor command frames over serial (115200 baud) from the Jetson ROS2 node.

## Build

Requires [PlatformIO](https://platformio.org/).

```bash
just build            # debug build
just build-release    # release build
```

Or with PlatformIO directly:

```bash
pio run -e debug
pio run -e release
```

## Upload

```bash
just upload           # upload debug build
just upload-release   # upload release build
```

## Monitor

```bash
just monitor
```

## Motor Pinout

| Motor          | Enable | Pulse | Direction | Default Dir |
|----------------|--------|-------|-----------|-------------|
| mid_foot       | 2      | 6     | 21        | -           |
| head           | 3      | 7     | 20        | -           |
| left_shoulder  | 4      | 8     | 19        | true        |
| right_shoulder | 4      | 9     | 18        | false       |
| left_foot      | 5      | 10    | 17        | false       |
| right_foot     | 5      | 11    | 16        | true        |

Note: left/right shoulder share enable pin 4; left/right foot share enable pin 5.

## Homing

On startup the head motor is automatically referenced using a hall effect sensor on pin 28 (homing speed 750 Hz, reference angle 300 degrees).
