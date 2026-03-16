from __future__ import annotations

from dataclasses import dataclass

from tracking.kalman_filter import KalmanFilter, KalmanState

import numpy as np


@dataclass
class Track:
    """
    One tracked object.

    This class owns:
    - a unique track_id
    - one Kalman filter state
    - bookkeeping for lifecycle management
    - one optional appearance embedding
    """
    track_id: int
    class_id: int
    score: float
    kf_state: KalmanState
    embedding: np.ndarray | None = None

    total_age_sec: float = 0.0
    hits: int = 1
    time_since_update_sec: float = 0.0
    confirmed: bool = False

    def predict(self, kf: KalmanFilter, dt: float) -> None:
        self.kf_state = kf.predict(self.kf_state, dt)
        self.total_age_sec += dt
        self.time_since_update_sec += dt

    def update(self, kf: KalmanFilter, cx: float, cy: float, w: float, h: float, score: float, embedding: np.ndarray | None = None) -> None:
        z = kf.measurement_from_bbox(cx, cy, w, h)
        self.kf_state = kf.update(self.kf_state, z)
        self.score = score
        self.hits += 1
        self.time_since_update_sec = 0.0

        if embedding is not None:
            self.embedding = embedding.astype(np.float32, copy=True)

    def bbox_xywh(self) -> tuple[float, float, float, float]:
        return KalmanFilter.bbox_from_state(self.kf_state)

    def bbox_xyxy(self) -> tuple[float, float, float, float]:
        cx, cy, w, h = self.bbox_xywh()
        x1 = cx - w / 2.0
        y1 = cy - h / 2.0
        x2 = cx + w / 2.0
        y2 = cy + h / 2.0
        return (x1, y1, x2, y2)