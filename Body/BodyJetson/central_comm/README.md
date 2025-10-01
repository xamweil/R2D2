# motor-control

## usage

to build the image and start up the service
```sh
docker compose up -d --build
```

the docker image build process installs the dependencies and builds the service using colcon

to stop and restart the container:
```sh
docker compose down
docker compose restart
```

to open a interactive shell on the container:
```sh
docker-compose exec motor-control bash
```

## controller setup

find the controller's unique identifiers:

```sh
# this will list all /dev/input/event* devices. find the controller
sudo evtest 
```

now run `sudo /dev/input/event5`. this will show information on the controller that should look like this:

```
Input driver version is 1.0.1
Input device ID: bus 0x5 vendor 0x54c product 0x9cc version 0x8100
Input device name: "Wireless Controller"
```

create a file `99-ps4-controller.rules` that looks like this:

```
# PlayStation 4 Controller - Main controller (buttons/sticks)
SUBSYSTEM=="input", ATTRS{id/vendor}=="054c", ATTRS{id/product}=="09cc", \
  ENV{ID_INPUT_JOYSTICK}=="1", KERNEL=="event*", \
  SYMLINK+="input/ps4-controller"

# PlayStation 4 Controller - Motion Sensors (gyro/accelerometer)
SUBSYSTEM=="input", ATTRS{id/vendor}=="054c", ATTRS{id/product}=="09cc", \
  ENV{ID_INPUT_ACCELEROMETER}=="1", KERNEL=="event*", \
  SYMLINK+="input/ps4-motion"

# PlayStation 4 Controller - Touchpad
SUBSYSTEM=="input", ATTRS{id/vendor}=="054c", ATTRS{id/product}=="09cc", \
  ENV{ID_INPUT_TOUCHPAD}=="1", KERNEL=="event*", \
  SYMLINK+="input/ps4-touchpad"
```

make sure to enter the correct vendor and product ids

copy the file to the to the udev rules dir:

```sh
sudo cp 99-ps4-controller.rules /etc/udev/rules.d/
```

reload udev:

```sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

running `ls -la /dev/input/ps4-*` should now look like this:

```sh
lrwxrwxrwx. 1 root root 7 Sep  3 12:04 /dev/input/ps4-controller -> event29
lrwxrwxrwx. 1 root root 7 Sep  3 12:04 /dev/input/ps4-motion -> event30
lrwxrwxrwx. 1 root root 7 Sep  3 12:04 /dev/input/ps4-touchpad -> event31
```

