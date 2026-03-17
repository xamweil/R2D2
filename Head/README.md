# Head

Electronics and control systems located inside the dome.

## Devices

### Raspberry Pi 5
- runs ROS camera node
- streams camera images to the Jetson
- handles communication with head electronics

### Arduino Uno
- controls servo motors
- operates the mechanical lids and nobs

### Seeeduino XIAO (LED Matrix Controller)
- drives the LED matrix displays
- receives animation commands from the Raspberry Pi