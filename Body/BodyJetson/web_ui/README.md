# R2D2 Web UI

Lightweight browser interface for monitoring and controlling the R2D2 robot.

The UI connects to the ROS2 backend through the `ui_bridge` service.

---

## Access

The Web UI runs on port:

`8000`

Example:

`http://192.168.66.2:8000`

---

## Features

* **Camera stream (MJPEG)**
* **Camera tilt control**
* **MPU sensor display**
* **Head actuator control**

  * lids
  * nobs
  * grouped actions

---

## Backend

The UI communicates with the ROS2 system via the `ui_bridge` node.

Main responsibilities:

* ROS topic subscriptions
* device service calls
* WebSocket robot state updates
* MJPEG camera streaming

---

## Reference

The UI uses the [**Star Jedi**](https://www.dafont.com/de/star-jedi.font) fonts:

```
Starjedi.ttf
STJEDISE.TTF
```

---

## To Do

* Implement head movement
* Implement 3D robot state graphic
