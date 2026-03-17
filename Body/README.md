# Body

Central system and motor control hardware.

## Devices

### Jetson Orin Nano
- main ROS2 system
- runs perception and tracking
- hosts the web control interface
- coordinates communication with all robot devices

### RP2040 Pico (Motor Controller)
- controls the main drive motor
- receives motor commands from the Jetson
- handles low level motor control