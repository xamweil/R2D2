# R2D2 Web UI

Browser-based interface for monitoring and controlling the R2D2 robot.

The Web UI connects to the robot's ROS2 system through the **`ui_bridge` backend node** and provides real-time visualization and control of robot components.

---

## Access

The Web UI runs on port:

```

8000

```

Example:

```

[http://192.168.66.2:8000](http://192.168.66.2:8000)

```

---

## Features

Current capabilities include:

- Live camera stream (MJPEG)
- Object detection overlay
- Camera tilt control
- Head rotation control
- Lid and nob actuator control
- Robot action panel
- MPU sensor display

---

## Architecture Overview

```

ROS2 System
│
▼
ui_bridge (backend)
│
├─ WebSocket → robot state updates
├─ HTTP POST → robot commands
└─ MJPEG → camera stream
│
▼
Browser Frontend

````

---

## Running the UI

Using the launch file (recommended):

```bash
just launch-bridge
```

Override parameters:

```bash
just launch-bridge port:=8080 mjpeg_fps:=15
```

Or launch directly:

```bash
ros2 launch ui_bridge_cpp ui_bridge_cpp.launch.py port:=9090 doc_root:=/path/to/frontend mjpeg_fps:=3
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `9090` | HTTP listen port |
| `doc_root` | `/home/ros/frontend` | Path to the frontend static files |
| `mjpeg_fps` | `3` | MJPEG stream frame rate |

### Docker

From the main robot repository:

```bash
docker compose up -d --build web_ui
```

---

## Project Structure

```
web_ui/
├── docker/                #container startup scripts
├── frontend/                #browser UI
└── ros2_ws/src/ui_bridge       #ROS ↔ Web bridge
                   
```

---

## Development Documentation

More detailed documentation can be found in:

* `frontend/README.md` — UI architecture and panels
* `ros2_ws/src/ui_bridge/README.md` — ROS bridge backend

---

## Fonts

The UI uses the **Star Jedi** font family.

Source:
[https://www.dafont.com/star-jedi.font](https://www.dafont.com/star-jedi.font)

Files located in:

```
frontend/assets/fonts/
```

---

## MJPEG_TEST_PATTERN

By default the `/mjpeg` endpoint streams live frames from the `/relay/camera/image_raw/compressed` ROS topic. If no camera is available and you want to verify the MJPEG pipeline, enable the built-in animated test pattern:

```bash
colcon build --packages-select ui_bridge_cpp --cmake-args -DMJPEG_TEST_PATTERN=ON
```

This compiles in `jpeg_generator.cpp` and the stb image library. Without the flag, neither is included in the binary.

---

## To Do

* Implement head movement automation
* Implement 3D robot state visualization
