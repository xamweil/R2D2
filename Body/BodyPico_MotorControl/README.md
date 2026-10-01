# BodyPico_MotorControl

Firmware for the Raspberry Pi Pico that performs low-level control of the body stepper motors.

## What it does

The Pico receives binary frames from the Jetson over serial and controls:

- middle foot motor
- head rotation motor
- left shoulder motor
- right shoulder motor

It handles:

- motor enable / direction / target angle / target velocity
- local motor update loop
- shoulder safety checks using forwarded IMU data

## Hardware

**Board:** Raspberry Pi Pico RP2040

### Motor pin mapping

| Motor | Enable | Step | Dir |
|------|--------|------|-----|
| middle foot | GP2 | GP6 | GP21 |
| head | GP3 | GP7 | GP20 |
| left shoulder | GP4 | GP8 | GP19 |
| right shoulder | GP4 | GP9 | GP18 |

Notes:

- enable is active low
- shoulder motors share the same enable pin
- the Jetson connects directly to the Pico over USB serial

## Serial protocol

The Pico accepts two frame types.

### 1) Motor frame

Start byte:

```txt
0xAA
````

Payload:

* control word (`u32`, little-endian)
* 6 motor value groups
* each value group = `float angle` + `uint8 velocity`

The Pico currently uses motor indices:

| Index | Motor          |
| ----- | -------------- |
| 0     | middle foot    |
| 1     | head           |
| 2     | left shoulder  |
| 3     | right shoulder |

### 2) IMU frame

Start byte:

```txt
0xAB
```

Payload contains 6 IMU samples in fixed order:

1. head
2. body
3. left foot
4. left leg
5. right foot
6. right leg

Each IMU sample contains:

* `int16 accel_x`
* `int16 accel_y`
* `int16 accel_z`
* `uint32 ts_ms`

If `ts_ms == 0`, that IMU is treated as invalid.

## Shoulder safety

Each shoulder motor receives references to the shared IMU store maintained by the serial processor.

Currently the shoulder logic uses:

* body IMU
* corresponding leg IMU

The relative angle is evaluated in the **YZ plane** using accelerometer data.

## Build

From this folder:

```bash
cd Body/BodyPico_MotorControl
pio run
```

## Flash

With the Pico connected by USB to the Jetson:

```bash
cd Body/BodyPico_MotorControl
pio run -t upload
```

## Monitor

```bash
cd Body/BodyPico_MotorControl
pio device monitor -b 115200
```
