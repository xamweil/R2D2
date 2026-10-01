
# LeftLegEsp32c3 – micro-ROS radar + taster

Firmware for a **Seeed XIAO ESP32-C3** in the **left leg**.


- read one **LD2410** radar module over UART
- detect two **taster** button press events
- publish both into the ROS 2 graph using **micro-ROS over Wi-Fi**

Published topics:

- `/leg_l/taster_event` (`leg_msg/msg/TasterEvent`)
- `/leg_l/radar` (`leg_msg/msg/Ld2410State`)

## What’s here

- `src/main.cpp` – top-level wiring, pin definitions, and loop
- `include/button.h`, `src/button.cpp` – debounced button press detection
- `include/radar.h`, `src/radar.cpp` – LD2410 handling
- `include/micro_ros_node.h`, `src/micro_ros_node.cpp` – micro-ROS Wi-Fi transport and publishers
- `include/secrets.h` – Wi-Fi credentials, not committed

## Hardware

- **Seeed XIAO ESP32-C3**
- **LD2410** radar module over UART
- two **taster** buttons using internal pull-ups
- Wi-Fi connection to the robot LAN

### Current pin usage

Defined at the top of `src/main.cpp`.

- `RADAR_RX_PIN 4` → radar TX
- `RADAR_TX_PIN 5` → radar RX
- `TASTER_1_PIN 6`
- `TASTER_2_PIN 7`

## ROS messages

### `leg_msg/msg/TasterEvent`

Published once per button press.

```text
std_msgs/Header header
uint8 button_id
````

### `leg_msg/msg/Ld2410State`

Published periodically.

```text
std_msgs/Header header

bool presence

uint16 moving_distance_cm
uint8 moving_energy

uint16 stationary_distance_cm
uint8 stationary_energy
```

## Behavior

### Taster buttons

* buttons use `INPUT_PULLUP`
* a press is detected on the falling edge after debounce
* only the **press event** is published
* no release event is sent

### Radar

* LD2410 is initialized over `HardwareSerial`
* latest values are polled continuously
* radar data is published periodically
* values published:

  * presence
  * moving distance + energy
  * stationary distance + energy

## Wi-Fi / micro-ROS

The ESP connects to Wi-Fi using credentials from `include/secrets.h`, then connects to the micro-ROS Agent on the Jetson.

Current agent target in `main.cpp`:

* `192.168.66.2:8888`

## Build

This project is built with **PlatformIO** in **WSL**.

Main settings are in `platformio.ini`.

Relevant dependencies:

* `micro_ros_platformio`
* `ncmreynolds/ld2410`

## Notes

* `leg_msg` is taken from the main ROS workspace as the single source of truth
* in this setup it is exposed to the firmware build through `extra_packages/leg_msg`

## Setup notes for environment

### Create `include/secrets.h`

```cpp
#pragma once

#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"
```

### Share USB device into WSL

Find the board in Windows PowerShell:

```powershell
usbipd list
```

Bind once as admin:

```powershell
usbipd bind --busid 3-2
```

Attach into WSL:

```powershell
usbipd attach --wsl --busid 3-2 --force
```

Verify in WSL:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

### Build / clean / flash

Clean micro-ROS build state:

```bash
sudo /root/.platformio/penv/bin/platformio run --target clean_microros
```

Build:

```bash
sudo /root/.platformio/penv/bin/platformio run
```

Flash:

```bash
sudo /root/.platformio/penv/bin/platformio run --target upload
```

Serial monitor:

```bash
sudo /root/.platformio/penv/bin/platformio device monitor
```

### Jetson micro-ROS Agent

Start with docker compose in `Body/BodyJetson`:

```bash
docker compose up -d --build micro_ros_agent
docker compose logs -f micro_ros_agent
```

### Jetson ROS checks

Check topics:

```bash
ros2 topic list | grep leg_l
```

Echo button events:

```bash
ros2 topic echo /leg_l/taster_event
```

Echo radar data:

```bash
ros2 topic echo /leg_l/radar
```


