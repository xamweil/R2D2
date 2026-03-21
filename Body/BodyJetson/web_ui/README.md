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

From the main robot repository:

```bash
docker compose up -d --build web_ui
````

The container will automatically:

1. build the ROS workspace if necessary
2. launch the `ui_bridge` node
3. start the web server

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

## To Do

* Implement head movement automation
* Implement 3D robot state visualization
