# Web UI Frontend

This directory contains the browser-based user interface for the R2D2 robot.

The frontend connects to the ROS system through the `ui_bridge` backend using:

- **WebSocket** for robot state updates
- **HTTP POST** for commands
- **MJPEG stream** for the camera feed

The UI is intentionally written in **plain JavaScript modules** with minimal dependencies to keep the system lightweight and easy to maintain.

---

# Directory Structure

```

frontend/
├── assets/
│   ├── fonts/       UI fonts
│   └── img/         images used by the interface
│
├── css/
│   ├── main.css        global styles
│   ├── layout.css      layout and grid system
│   └── components.css  panel and UI component styling
│                    
│
├── js/
│   ├── api/         backend communication
│   ├── state/       shared UI state
│   ├── ui/          UI panels and components
│   └── main.js      application entry point
│
└── index.html       main UI page

```

---

# Application Flow

```

ROS Topics
│
▼
ui_bridge backend
│
▼
WebSocket robot state message
│
▼
main.js
│
▼
UI panels update

```

Robot state is pushed to the browser through a **WebSocket connection**.

Each UI module reads the relevant information from the state and updates the DOM.

---

# Main Entry Point

The application starts in:

```

js/main.js

```

Responsibilities:

- initialize panels
- open the WebSocket connection
- distribute robot state updates
- coordinate UI modules

Typical initialization flow:

```

initCameraControlsPanel()
initCameraPanel()
initHeadControlPanel()
initControlsPanel()
initMpuPanel()

```

---

# UI Panels

UI logic is split into modular panel controllers located in:

```

js/ui/

```

Current panels:

| File | Purpose |
|-----|------|
| cameraPanel.js | camera stream and detection overlay |
| cameraControlsPanel.js | camera tilt controls |
| headControlPanel.js | head rotation interface |
| controlsPanel.js | lids, nobs, grouped actions |
| mpuPanel.js | MPU sensor display |
| statusBar.js | UI status indicators |

Each panel module:

- attaches DOM listeners
- renders UI updates
- reacts to state changes

---

# HTML Templates

Reusable UI components are defined using `<template>` elements in `index.html`.

Example templates:

```

mpu-card-template
lid-card-template
nob-card-template
action-button-template
head-toggle-group-template
head-input-row-template

```

Templates are cloned dynamically by the UI modules.

This keeps `index.html` clean and avoids large static HTML blocks.

---

# Camera Rendering Pipeline

The camera system uses two elements:

```

camera-image
camera-overlay

```

Structure:

```

camera-frame
├── camera-image   (MJPEG stream)
└── camera-overlay (bounding boxes)

```

Detection pipeline:

```

/scene_understanding/detections
│
▼
WebSocket state update
│
▼
updateSceneDetections()
│
▼
cameraPanel.js
│
▼
renderDetectionOverlay()

```

Bounding boxes are drawn as positioned DOM elements inside `camera-overlay`.

---

# Detection Coordinate Transformation

The perception node runs YOLO inference in a **640x640 model space**.

The camera image uses **letterboxing**:

```

640x360 image
↓
letterboxed
↓
640x640 model input

```

The UI removes the padding and scales the detections back into the displayed image.

This allows the overlay to remain correct even if the camera resolution changes.

---

# Backend Communication

Frontend API helpers live in:

```

js/api/

```

### WebSocket

```

websocket.js

```

Handles:

- robot state stream
- automatic reconnect
- message dispatch

---

### Device commands

```

deviceService.js

```

Calls backend service:

```

/device_command

```

Used for:

- lids
- nobs
- grouped actions

---

### Motor commands

```

motorCommandService.js

```

Publishes to ROS topic:

```

/motor_command

```

Used for:

- head rotation
- camera tilt

---

# Adding a New UI Panel

Typical workflow:

### 1 Create panel module

```

js/ui/myPanel.js

```

### 2 Add HTML template

```

index.html

```

### 3 Initialize in main.js

```

initMyPanel()

````

### 4 Subscribe to robot state if needed

Example:

```javascript
payload.entries.my_topic
````

---

# Adding a New Robot Control

1. create UI control
2. call backend API

Example:

```
callDeviceCommand(device, method, args)
```

or

```
publishMotorCommand(...)
```

---

# Styling

Styling is split into three files:

```
main.css
layout.css
components.css
```

Purpose:

| File           | Role          |
| -------------- | ------------- |
| main.css       | global theme  |
| layout.css     | grid layout   |
| components.css | panel styling |

---

# Fonts

The UI uses the **Star Jedi** font family.

Located in:

```
assets/fonts/
```

**Credits**:

Star Jedi by [Boba Fonts](https://www.dafont.com/star-jedi.font)


---

### When adding new state entries

State entries appear in the WebSocket payload as:

```
payload.entries.<alias>
```

The alias is defined in the backend configuration.

See:

```
ui_bridge/config.py
```

---

# Related Documentation

Backend ROS bridge documentation:

```
ros2_ws/src/ui_bridge/README.md
```



