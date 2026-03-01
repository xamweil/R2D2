## ros-camera (ROS2 camera node)

### What it does

* Runs `camera_ros` (`camera_node`) to capture frames via **libcamera** and publish them in ROS2.
* Publishes:

  * `/<CAM_NODE_NAME>/image_raw` (`sensor_msgs/Image`)
  * `/<CAM_NODE_NAME>/camera_info` (`sensor_msgs/CameraInfo`)
  * (commonly) `/<CAM_NODE_NAME>/image_raw/compressed` for WLAN-friendly streaming

### Quick ROS2 checks (node-level)

* List camera topics:

```bash
ros2 topic list | grep -E "image|camera_info"
```

* Confirm it’s publishing:

```bash
ros2 topic echo --once /camera/image_raw/header
```

* Measure FPS:

```bash
ros2 topic hz /camera/image_raw
```

* View one image stream (compressed recommended):

```bash
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw
# or (if available in your setup)
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw/compressed
```

### Configure resolution / FPS (startup params)

* Set these (via your container env -> passed as ROS params):

  * `CAM_WIDTH` (e.g. `1280`)
  * `CAM_HEIGHT` (e.g. `720`)
  * `CAM_FPS` (e.g. `30`)
  * `CAM_NODE_NAME` (default `camera`)
  * `CAM_FRAME_ID` (default `camera_optical_frame`)

### References

* `camera_ros`: [https://github.com/christianrauch/camera_ros](https://github.com/christianrauch/camera_ros)
* `raspberrypi/libcamera`: [https://github.com/raspberrypi/libcamera](https://github.com/raspberrypi/libcamera)
