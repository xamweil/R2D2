from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np
from scipy.optimize import linear_sum_assignment

from tracking.kalman_filter import KalmanFilter
from tracking.track import Track


@dataclass
class Detection:
    cx: float
    cy: float
    w: float
    h: float
    score: float
    class_id: int

    def bbox_xywh(self) -> tuple[float, float, float, float]:
        return (self.cx, self.cy, self.w, self.h)

    def bbox_xyxy(self) -> tuple[float, float, float, float]:
        x1 = self.cx - self.w / 2.0
        y1 = self.cy - self.h / 2.0
        x2 = self.cx + self.w / 2.0
        y2 = self.cy + self.h / 2.0
        return (x1, y1, x2, y2)


class ByteTracker:
    """
    Lightweight ByteTrack-style tracker.

    Core idea:
    - split detections into high- and low-confidence groups
    - first match tracks to high-confidence detections
    - then match remaining tracks to low-confidence detections
    - create new tracks only from unmatched high-confidence detections
    """

    def __init__(self, track_class_ids: list[int] | None = None, high_thresh: float = 0.6, low_thresh: float = 0.1,
        iou_thresh: float = 0.3, max_time_since_update: int = 10, ) -> None:
        self.kf = KalmanFilter()

        self.track_class_ids = set(track_class_ids or [0])
        self.high_thresh = high_thresh
        self.low_thresh = low_thresh
        self.iou_thresh = iou_thresh
        self.max_time_since_update = max_time_since_update

        self.tracks: List[Track] = []
        self.next_track_id = 1

    def set_control_input(self, u_x: float, u_y: float) -> None:
        self.kf.set_control_input(u_x, u_y)

    def update(self, detections: list[Detection], dt: float) -> list[Track]:
        """
        Main update step for one frame.
        Returns active confirmed tracks.
        """
        detections = self._filter_detections(detections)

        high_dets = [d for d in detections if d.score >= self.high_thresh]
        low_dets = [d for d in detections if self.low_thresh <= d.score < self.high_thresh]

        # Predict all existing tracks
        for track in self.tracks:
            track.predict(self.kf, dt)

        # First association: tracks <-> high confidence detections
        unmatched_tracks, unmatched_high_dets = self._associate_and_update(
            self.tracks,
            high_dets,
        )

        # Second association: remaining tracks <-> low confidence detections
        if unmatched_tracks and low_dets:
            unmatched_tracks, _ = self._associate_and_update(
                unmatched_tracks,
                low_dets,
            )

        # Create new tracks from unmatched HIGH confidence detections only
        for det in unmatched_high_dets:
            self._start_new_track(det)

        # Remove stale tracks
        self.tracks = [
            t for t in self.tracks
            if t.time_since_update <= self.max_time_since_update
        ]

        # Return only confirmed and recently updated tracks
        active_tracks = [
            t for t in self.tracks
            if t.confirmed and t.time_since_update == 0
        ]
        return active_tracks

    def _filter_detections(self, detections: list[Detection]) -> list[Detection]:
        return [d for d in detections if d.class_id in self.track_class_ids and d.score >= self.low_thresh]

    def _start_new_track(self, det: Detection) -> None:
        state = self.kf.initiate(det.cx, det.cy, det.w, det.h)
        track = Track(
            track_id=self.next_track_id,
            class_id=det.class_id,
            score=det.score,
            kf_state=state,
        )
        self.tracks.append(track)
        self.next_track_id += 1

    def _associate_and_update(self, tracks: list[Track], detections: list[Detection], ) -> tuple[list[Track], list[Detection]]:
        if not tracks or not detections:
            return tracks[:], detections[:]

        cost_matrix = self._build_iou_cost_matrix(tracks, detections)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        matched_track_indices = set()
        matched_det_indices = set()

        for r, c in zip(row_ind, col_ind):
            iou = 1.0 - cost_matrix[r, c]
            if iou < self.iou_thresh:
                continue

            track = tracks[r]
            det = detections[c]

            track.update(
                self.kf,
                det.cx,
                det.cy,
                det.w,
                det.h,
                det.score,
            )

            matched_track_indices.add(r)
            matched_det_indices.add(c)

        unmatched_tracks = [
            track for i, track in enumerate(tracks)
            if i not in matched_track_indices
        ]
        unmatched_dets = [
            det for i, det in enumerate(detections)
            if i not in matched_det_indices
        ]

        return unmatched_tracks, unmatched_dets

    def _build_iou_cost_matrix(self, tracks: list[Track], detections: list[Detection], ) -> np.ndarray:
        cost = np.ones((len(tracks), len(detections)), dtype=np.float32)

        for i, track in enumerate(tracks):
            tx1, ty1, tx2, ty2 = track.bbox_xyxy()

            for j, det in enumerate(detections):
                dx1, dy1, dx2, dy2 = det.bbox_xyxy()
                iou = self._iou_xyxy((tx1, ty1, tx2, ty2), (dx1, dy1, dx2, dy2))
                cost[i, j] = 1.0 - iou

        return cost

    @staticmethod
    def _iou_xyxy(a: tuple[float, float, float, float], b: tuple[float, float, float, float], ) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)

        union = area_a + area_b - inter_area
        if union <= 0.0:
            return 0.0

        return inter_area / union