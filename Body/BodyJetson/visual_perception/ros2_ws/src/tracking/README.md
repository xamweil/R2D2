# tracking

This package implements **real-time multi-object tracking with appearance re-identification**.

It associates detections across frames and maintains stable identities.

The implementation is inspired by:

[**ByteTrack**](https://arxiv.org/pdf/2110.06864)

---

# Node

```
tracking_node
```

Responsibilities:

1. receive detections
2. estimate object motion
3. associate detections to tracks
4. maintain identities
5. compute appearance embeddings

---

# Architecture

```
Detections
   │
   ▼
ByteTracker
   │
   ├── Kalman Filter (motion prediction)
   │
   ├── IoU association
   │
   └── ReID embedding matching
   │
   ▼
Tracks
```

---

# Tracking Components

### Kalman Filter

Implemented in:

```
kalman_filter.py
```

State vector:

```
[cx, cy, w, h, vx, vy]
```

Provides:

* motion prediction
* measurement update
* noise modeling

---

### ByteTracker

File:

```
byte_tracker.py
```

Implements the association logic described in the ByteTrack paper.

Algorithm:

1. predict track positions
2. match tracks with **high confidence detections**
3. match remaining tracks with **low confidence detections**
4. create new tracks
5. remove stale tracks

Matching uses **Hungarian assignment** based on IoU distance.

---

# Re-Identification

File:

```
osnet_embedder.py
```

Embedding model:

```
OSNet x0.25
```

Produces:

```
512-dimensional appearance vector
```

Embedding generation pipeline:

```
detection crop
 → preprocessing
 → TensorRT OSNet inference
 → normalized embedding
```

Embeddings allow tracks to be recovered after short occlusions.

---

# Track Lifecycle Model

Each track stores:

```
track_id
kf_state
hits
age
time_since_update
embedding
```

Track confirmation requires a minimum number of updates.

The tracker uses an **adaptive lifetime model**.

Concept:

Tracks that have existed longer are trusted more and allowed to survive longer without detections.

Lost timeout is based on:

```
track_age
```

Older tracks remain in memory longer, enabling recovery after temporary occlusions.

---

# Association Strategy

Matching uses two signals:

### 1. Motion similarity

Measured via:

```
IoU(track_bbox, detection_bbox)
```

### 2. Appearance similarity

Measured via cosine similarity between embeddings.

This combination improves robustness when objects overlap or cross paths.

---

# ROS Interface

Input:

```
/scene_understanding/detections
/relay/camera/image_raw
/motor_command
```

Output:

```
/tracking/tracks
```

---

# Performance

The tracker is optimized for real-time operation.

Key optimizations:

* numpy thread limits
* minimal memory allocations
* TensorRT embeddings
* contiguous image buffers

Target:

```
30 FPS real-time tracking
```

---

# Important Parameters

Key parameters exposed via ROS:

```
high_thresh
low_thresh
iou_thresh
min_confirmed_hits
max_time_since_update_sec
max_unconfirmed_age_sec
```

These parameters strongly influence tracking stability.

---


## To do

* global identity memory
* long-term re-identification



---
