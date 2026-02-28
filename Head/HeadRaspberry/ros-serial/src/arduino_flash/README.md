# Flash arduino
Go in the container 
```sh
cd R2D2/Head/HeadRaspberry/ros-serial
just sh
cd ros2_ws
```
Make sure flash node is running in the container ```ros-serial```.

```sh
source install/setup.bash
ros2 run arduino_flash flash_node &
```
Run the flash. It will flash wahtever is under ```HeadUno```.

```sh
ros2 service call /flash flash_msg/srv/FlashCommand "{name: 'Uno'}"
```
<br><br>
-
<br>
Look for package

```sh
ros2 pkg list | grep arduino_flash
```