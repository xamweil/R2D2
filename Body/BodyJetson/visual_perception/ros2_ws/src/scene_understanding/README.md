# scene_understanding

This package performs **real-time object detection** using a TensorRT-optimized YOLOv8 model.

It is the **first stage of the perception pipeline**.

---

# Node Overview

Node:

```
scene_understanding_node
```

Responsibilities:

1. receive camera images
2. run YOLOv8 TensorRT inference
3. decode detections
4. publish detection messages

---

# Pipeline

```
Image
 │
 ▼
ImagePreprocessor
 │
 ▼
TensorRT Engine
 │
 ▼
YOLO Decoder
 │
 ▼
Detection Message
```

Modules:

| Module                   | Responsibility                              |
| ------------------------ | ------------------------------------------- |
| image_preprocessor       | resize / normalize images                   |
| tensor_rt_engine         | TensorRT inference wrapper                  |
| yolo_decoder             | converts raw model output to bounding boxes |
| scene_understanding_node | ROS integration                             |

---

# Input

Subscribed topic:

```
/relay/camera/image_raw
```

Image format:

```
BGR8
```

Typical resolution:

```
640x360
```

Resolution can change depending on camera configuration.

---

# Output

Published topic:

```
/scene_understanding/detections
```

Each detection contains:

```
[cx,
cy,
width,
height,
score,
class_id]
```

Coordinates are in **image pixel space**.

---

# Model

Detector: ``YOLOv8n``

Converted pipeline:

```
PyTorch
 -> ONNX
 -> TensorRT
```

TensorRT engine:

```
/home/ros/models/yolov8n/yolov8n.engine
```

---

# Swapping the Model

To replace the detector:

1. add new model directory under

```
models/
```

2. create

```
model.yaml
```

3. implement conversion in

```
prepare_models.py
```

4. update engine path in node parameters

---

# Debugging

Common issues:

### Engine not loading

Check:

```
/home/ros/models/yolov8n/yolov8n.engine
```

Rebuild:

```
python3 /home/ros/models/prepare_models.py --build yolov8n
```

---

### Incorrect detections

Check preprocessing:

```
image_preprocessor.py
```

YOLO models are sensitive to input scaling.

---

# Performance Notes

Inference runs entirely on GPU.

Typical latency on Jetson Orin:

```
~5–8 ms
```

The node is designed to support **30 FPS real-time operation**.

---

# Future Improvements

Potential upgrades:

* face recognition integration
* multi-class detection


---
