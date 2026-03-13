
---

# Camera Relay (Jetson)

This container relays the camera stream from the Raspberry Pi to the Jetson ROS network and provides both **compressed** and **decoded raw images** for downstream nodes.

## Topics

| Input                          | Output                               |
| ------------------------------ | ------------------------------------ |
| `/camera/image_raw/compressed` | `/relay/camera/image_raw/compressed` |
| `/camera/camera_info`          | `/relay/camera/camera_info`          |
|                                | `/relay/camera/image_raw`            |

### Output descriptions

| Topic                                | Type                          | Purpose                             |
| ------------------------------------ | ----------------------------- | ----------------------------------- |
| `/relay/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Forwarded JPEG stream               |
| `/relay/camera/image_raw`            | `sensor_msgs/Image`           | Decoded raw frames for vision nodes |
| `/relay/camera/camera_info`          | `sensor_msgs/CameraInfo`      | Camera calibration data             |

---

# Nodes

## `camera_stream_relay`

Subscribes to the compressed camera stream and:

1. republishes the compressed image
2. decodes JPEG frames
3. publishes raw images

```
/camera/image_raw/compressed
        │
        ▼
camera_stream_relay
        │
        ├──► /relay/camera/image_raw/compressed
        └──► /relay/camera/image_raw
```

---

## `camera_info_relay`

Forwards camera calibration:

```
/camera/camera_info
        │
        ▼
camera_info_relay
        │
        └──► /relay/camera/camera_info
```

---

# QoS Configuration

The relay uses the following QoS strategy.

### Subscriptions

```
SensorDataQoS()
```

This ensures compatibility with typical camera drivers which often publish with **BEST_EFFORT**.

### Publishers

```
RELIABLE
KEEP_LAST
```

This guarantees compatibility with downstream nodes such as the web UI.

---

# Development / Debugging

### Check upstream camera

```
ros2 topic hz /camera/image_raw/compressed
```

### Check relay output

```
ros2 topic hz /relay/camera/image_raw/compressed
```

### Check decoded frames

```
ros2 topic hz /relay/camera/image_raw
```

---

# Why this exists

Raw images are large and expensive to transmit over the robot network.

Instead the system sends **JPEG images from the Raspberry Pi** and decodes them **locally on the Jetson**.

Benefits:

* reduced network bandwidth
* Jetson handles image decoding
* multiple local consumers can access the stream
* camera namespace is clean (`/relay/camera/...`)

---

