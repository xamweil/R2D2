# visual_perception

This container provides the **visual perception stack of the Robot**.
It performs **real-time scene understanding and multi-object tracking** using GPU-accelerated neural networks.

The stack currently detects people, tracks them across frames, and maintains identity using appearance embeddings.

The system is designed to run **real-time on NVIDIA Jetson Orin** using **TensorRT optimized models**.

---

# System Overview

Pipeline:

```
Camera (/relay/camera/image_raw)
        │
        ▼
scene_understanding_node
(YOLOv8 detection)
        │
        ▼
/scene_understanding/detections
        │
        ▼
tracking_node
(ByteTrack + Kalman Filter + OSNet ReID)
        │
        ▼
/tracking/tracks
```

Main components:

| Component                 | Description                                   |
| ------------------------- | --------------------------------------------- |
| scene_understanding       | Object detection using YOLOv8 TensorRT        |
| tracking                  | Multi-object tracking with ByteTrack and ReID |
| models                    | Model fetch + conversion pipeline             |
| visual_perception_bringup | Launch configuration                          |

---

# ROS Nodes

### scene_understanding_node

Runs YOLOv8 detection.

Publishes:

```
/scene_understanding/detections
```

---

### tracking_node

Performs multi-object tracking.

Features:

* Kalman filter motion model
* ByteTrack association
* Re-identification embeddings (OSNet)
* adaptive track lifetime model

Publishes:

```
/tracking/tracks
```

---

# Models

Models are stored under:

```
/home/ros/models
```

Weights are **not stored in git**.

They are automatically downloaded when the container starts.

Model fetching is handled by:

```
models/fetch_models.py
```

Model preparation (ONNX + TensorRT engine build):

```
models/prepare_models.py
```

---

## YOLOv8 Detector

Model:

```
YOLOv8n
```

Artifacts:

```
yolov8n.onnx
yolov8n.engine
```

Used for real-time object detection.

---

## Re-Identification Model

Model: *OSNet x0.25*

Training dataset: MSMT17

Paper: [Omni-Scale](https://arxiv.org/abs/1905.00953) Feature Learning for Person Re-Identification

Artifacts:


osnet_x0_25.onnx
osnet_x0_25.engine


Used to generate **512-D appearance embeddings**.

---

# Container Startup

Startup is handled via:

```
docker/auto_launch.sh
```

Startup sequence:

1. fetch missing model weights
2. build TensorRT engines if missing
3. launch ROS perception stack

---

# Performance Notes

Key optimizations:

* TensorRT inference
* FP16 precision
* thread limiting for numpy / OpenBLAS
* contiguous memory handling for OpenCV

Environment variables used:

```
OMP_NUM_THREADS=1
OPENBLAS_NUM_THREADS=1
MKL_NUM_THREADS=1
NUMEXPR_NUM_THREADS=1
```

These prevent uncontrolled thread spawning which can otherwise consume multiple CPU cores.

---

# Configuration

Main runtime parameters are configured via ROS parameters.

Example:

```
track_class_ids
high_thresh
low_thresh
iou_thresh
reid_engine_path
```

---

# Repository Structure

```
visual_perception
│
├── docker
│   ├── auto_launch.sh
│   └── ros_entrypoint.sh
│
├── models
│   ├── fetch_models.py
│   ├── prepare_models.py
│   ├── yolov8n
│   └── osnet_x0_25
│
└── ros2_ws
    └── src
        ├── scene_understanding
        ├── tracking
        └── visual_perception_bringup
```

---

# Future Development

Planned perception features:

* face recognition
* pose estimation
* gesture recognition
* activity recognition
* long-term identity memory
* depth integration

The architecture is intentionally modular to allow additional perception nodes to be integrated.

---
