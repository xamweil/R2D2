
# UI Bridge (ROS ↔ Web Interface)

The **ui_bridge** package connects the robot's ROS2 system to the browser-based Web UI.

It exposes ROS topics, services, and commands through a lightweight web API so they can be consumed by the frontend.

The bridge is implemented as a ROS2 node combined with a **FastAPI web server**.

---

# Responsibilities

The `ui_bridge` performs four main tasks:

### 1. ROS Topic Subscriptions

Selected ROS topics are subscribed and their latest values are stored.

These values are periodically sent to the frontend through a **WebSocket robot state stream**.

Example topics:

```

/Body/mpu
/leg_l/imu/foot
/scene_understanding/detections

```

---

### 2. Robot Commands

The UI can trigger robot commands via HTTP requests.

Two types currently exist:

#### Device Commands

Used for components like:

- lids
- nobs
- grouped actions

These call the ROS service:

```

serial_msg/srv/DeviceCommand

```

---

#### Motor Commands

Used for direct actuator control:

```

/motor_command

```

Example usage:

- head rotation
- camera tilt

---

### 3. Camera Streaming

The camera image is streamed to the browser as **MJPEG**.

Source topic:

```

/relay/camera/image_raw/compressed

```

Pipeline:

```

ROS CompressedImage
│
▼
ui_bridge subscription
│
▼
MJPEG HTTP endpoint
│
▼
browser <img> stream

```

The frontend loads the stream from:

```

/mjpeg

```

---

### 4. Robot State WebSocket

Robot state is pushed to the browser through a WebSocket connection.

Endpoint:

```

/ws

````

Message structure:

```json
{
  "entries": {
    "imu_body": {...},
    "scene_detections": {...},
    "imu_left_leg": {...}
  }
}
````

Each entry contains:

```
present
stale
age
timestamp
data
```

This allows the UI to detect stale or missing sensors.

---

# Package Structure

```
ui_bridge/
├── api_node.py
│
├── config.py
│   topic alias configuration
│
├── ros_bridge.py
│   ROS subscriptions and publishers
│
├── ros_introspection.py
│   ROS type inspection utilities
│
├── ros_types.py
│   ROS message helpers
│
├── serialize.py
│   ROS → JSON serialization
│
└── web_app.py
    FastAPI server
```

---

# Topic Alias System

The UI does not directly reference ROS topic names.

Instead, topics are mapped through **aliases** defined in:

```
ui_bridge/config.py
```

Example:

```python
ALLOWED_TOPICS = {
    "imu_body": {
        "name": "/Body/mpu",
        "type": "tcp_msg/msg/MPU6500Sample"
    }
}
```

The alias becomes the key used by the frontend:

```
payload.entries.imu_body
```

This abstraction allows ROS topic names to change without modifying the frontend.

---

# Adding a New Topic

1. Add the topic to `ALLOWED_TOPICS` in:

```
ui_bridge/config.py
```

Example:

```python
"scene_detections": {
    "name": "/scene_understanding/detections",
    "type": "vision_msgs/msg/Detection2DArray"
}
```

2. Restart the container or rebuild the workspace.

3. The topic will automatically appear in the WebSocket robot state.

Frontend access example:

```javascript
payload.entries.scene_detections
```

---

# Serialization

ROS messages are converted into JSON using utilities in:

```
serialize.py
```

The serializer supports:

* standard ROS messages
* nested message structures
* arrays
* custom interface packages

This allows complex messages like:

```
vision_msgs/Detection2DArray
```

to be safely sent to the browser.

---

# Camera Streaming Implementation

The camera system uses a **lazy subscription model**.

When the frontend requests `/mjpeg`:

1. `ui_bridge` subscribes to the camera topic
2. frames are converted to MJPEG
3. the HTTP stream begins

When the browser disconnects:

```
Camera MJPEG unsubscribed
```

The ROS subscription is removed.

This prevents unnecessary bandwidth usage.

---

# Running the Bridge

The bridge is started automatically by the container.

Startup flow:

```
ros_entrypoint.sh
        │
        ▼
auto_launch.sh
        │
        ▼
colcon build (if needed)
        │
        ▼
ros2 run ui_bridge api_node
```

The web server starts on:

```
0.0.0.0:8000
```

---

# Development Tips

### Inspect topics

Use ROS tools to verify topics before adding them:

```
ros2 topic list
ros2 topic echo <topic>
ros2 topic info <topic>
```

---

### Inspect interface types

```
ros2 interface show <type>
```

Example:

```
ros2 interface show vision_msgs/msg/Detection2DArray
```

---

### Debug WebSocket messages

Open the browser developer console and inspect:

```
payload.entries
```

to see all received robot state entries.

---

# Related Documentation

Frontend UI architecture:

```
frontend/README.md
```

System overview:

```
web_ui/README.md
```



