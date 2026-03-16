#!/usr/bin/env python3
from __future__ import annotations

import math
import cv2

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from serial_msg.msg import MotorCommand
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from tracking.byte_tracker import ByteTracker, Detection
from tracking.osnet_embedder import OSNetEmbedder


class TrackingNode(Node):
    def __init__(self) -> None:
        super().__init__("tracking")

        self.declare_parameter("detection_topic", "/scene_understanding/detections")
        self.declare_parameter("tracks_topic", "/tracking/tracks")
        self.declare_parameter("motor_command_topic", "/motor_command")
        self.declare_parameter("track_class_ids", [0])
        self.declare_parameter("high_thresh", 0.6)
        self.declare_parameter("low_thresh", 0.1)
        self.declare_parameter("iou_thresh", 0.3)
        self.declare_parameter("max_time_since_update_sec", 2.0)
        self.declare_parameter("min_confirmed_hits", 4)
        self.declare_parameter("max_unconfirmed_age_sec", 0.2)
        self.declare_parameter("min_lost_time_sec", 2.0)
        self.declare_parameter("max_lost_time_sec", 15.0)
        self.declare_parameter("lost_time_tau_sec", 30.0)
        self.declare_parameter("image_topic", "/relay/camera/image_raw")
        self.declare_parameter("reid_engine_path", "/home/ros/models/osnet_x0_25/osnet_x0_25.engine")
        self.declare_parameter("image_buffer_size", 10)

        detection_topic = self.get_parameter("detection_topic").get_parameter_value().string_value
        tracks_topic = self.get_parameter("tracks_topic").get_parameter_value().string_value
        motor_command_topic = self.get_parameter("motor_command_topic").get_parameter_value().string_value
        track_class_ids = list(self.get_parameter("track_class_ids").value)
        high_thresh = float(self.get_parameter("high_thresh").value)
        low_thresh = float(self.get_parameter("low_thresh").value)
        iou_thresh = float(self.get_parameter("iou_thresh").value)
        max_time_since_update_sec = float(self.get_parameter("max_time_since_update_sec").value)
        min_confirmed_hits = int(self.get_parameter("min_confirmed_hits").value)
        max_unconfirmed_age_sec = float(self.get_parameter("max_unconfirmed_age_sec").value)
        min_lost_time_sec = float(self.get_parameter("min_lost_time_sec").value)
        max_lost_time_sec = float(self.get_parameter("max_lost_time_sec").value)
        lost_time_tau_sec = float(self.get_parameter("lost_time_tau_sec").value)
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        reid_engine_path = self.get_parameter("reid_engine_path").get_parameter_value().string_value
        image_buffer_size = int(self.get_parameter("image_buffer_size").value)

        # Default camera-motion control input u_k = [u_x, u_y]
        self.u_k_x = 0.0
        self.u_k_y = 0.0

        self._last_stamp_sec: float | None = None

        self.tracker = ByteTracker(
            track_class_ids=track_class_ids,
            high_thresh=high_thresh,
            low_thresh=low_thresh,
            iou_thresh=iou_thresh,
            max_time_since_update_sec=max_time_since_update_sec,
            min_confirmed_hits=min_confirmed_hits,
            max_unconfirmed_age_sec=max_unconfirmed_age_sec,
            min_lost_time_sec=min_lost_time_sec,
            max_lost_time_sec=max_lost_time_sec,
            lost_time_tau_sec=lost_time_tau_sec,
        )

        self.bridge = CvBridge()
        self.embedder = OSNetEmbedder(reid_engine_path)

        self.image_buffer_size = image_buffer_size
        self.image_buffer: dict[tuple[int, int], any] = {}

        self._detections_sub = self.create_subscription(
            Detection2DArray,
            detection_topic,
            self.detections_callback,
            10,
        )

        self._motor_command_sub = self.create_subscription(
            MotorCommand,
            motor_command_topic,
            self.motor_command_callback,
            10,
        )

        self._tracks_pub = self.create_publisher(
            Detection2DArray,
            tracks_topic,
            10,
        )

        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )

        self.get_logger().info("tracking started")
        self.get_logger().info(f"  detection_topic: {detection_topic}")
        self.get_logger().info(f"  tracks_topic: {tracks_topic}")
        self.get_logger().info(f"  motor_command_topic: {motor_command_topic}")
        self.get_logger().info(f"  track_class_ids: {track_class_ids}")
        self.get_logger().info(f"  initial u_k: [{self.u_k_x}, {self.u_k_y}]")
        self.get_logger().info(f"  image_topic: {image_topic}")
        self.get_logger().info(f"  reid_engine_path: {reid_engine_path}")

    def motor_command_callback(self, msg: MotorCommand) -> None:
        """
        Placeholder for later camera motion handling.

        """
        _ = msg
        self.u_k_x = 0.0
        self.u_k_y = 0.0

    def _get_buffered_image(self, msg: Detection2DArray):
        key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        return self.image_buffer.get(key, None)

    def _crop_detection(self, image_bgr, cx: float, cy: float, w: float, h: float):
        img_h, img_w = image_bgr.shape[:2]

        cx_px = cx * img_w
        cy_px = cy * img_h
        w_px = w * img_w
        h_px = h * img_h

        x1 = int(round(cx_px - w_px / 2.0))
        y1 = int(round(cy_px - h_px / 2.0))
        x2 = int(round(cx_px + w_px / 2.0))
        y2 = int(round(cy_px + h_px / 2.0))

        x1 = max(0, min(img_w, x1))
        y1 = max(0, min(img_h, y1))
        x2 = max(0, min(img_w, x2))
        y2 = max(0, min(img_h, y2))

        if x2 <= x1 or y2 <= y1:
            return None

        return np.ascontiguousarray(image_bgr[y1:y2, x1:x2])

    def image_callback(self, msg: Image) -> None:
        key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))

        image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.image_buffer[key] = image_bgr

        if len(self.image_buffer) > self.image_buffer_size:
            oldest_key = sorted(self.image_buffer.keys())[0]
            del self.image_buffer[oldest_key]

    def detections_callback(self, msg: Detection2DArray) -> None:
        stamp_sec = self._stamp_to_seconds(msg)

        if self._last_stamp_sec is None:
            dt = 1.0 / 30.0
        else:
            dt = stamp_sec - self._last_stamp_sec
            if not math.isfinite(dt) or dt <= 0.0:
                dt = 1.0 / 30.0

        self._last_stamp_sec = stamp_sec

        image_bgr = self._get_buffered_image(msg)
        detections = self._convert_ros_detections(msg, image_bgr)

        self.tracker.set_control_input(self.u_k_x, self.u_k_y)
        active_tracks = self.tracker.update(detections, dt)

        tracks_msg = Detection2DArray()
        tracks_msg.header = msg.header

        for track in active_tracks:
            cx, cy, w, h = track.bbox_xywh()

            det_msg = Detection2D()

            # If the installed vision_msgs version exposes an id field, use it.
            if hasattr(det_msg, "id"):
                det_msg.id = str(track.track_id)

            bbox = BoundingBox2D()
            bbox.center.position.x = float(cx)
            bbox.center.position.y = float(cy)
            bbox.size_x = float(w)
            bbox.size_y = float(h)
            det_msg.bbox = bbox

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(track.class_id))
            hypothesis.hypothesis.score = float(track.score)
            det_msg.results.append(hypothesis)

            tracks_msg.detections.append(det_msg)

        self._tracks_pub.publish(tracks_msg)

    def _convert_ros_detections(self, msg: Detection2DArray, image_bgr) -> list[Detection]:
        detections: list[Detection] = []

        for det_msg in msg.detections:
            if not det_msg.results:
                continue

            best = max(det_msg.results, key=lambda r: float(r.hypothesis.score))

            try:
                class_id = int(best.hypothesis.class_id)
            except ValueError:
                continue

            score = float(best.hypothesis.score)
            cx = float(det_msg.bbox.center.position.x)
            cy = float(det_msg.bbox.center.position.y)
            w = float(det_msg.bbox.size_x)
            h = float(det_msg.bbox.size_y)

            embedding = None
            if image_bgr is not None and class_id == 0:
                crop = self._crop_detection(image_bgr, cx, cy, w, h)
                if crop is not None:
                    try:
                        embedding = self.embedder.embed(crop)
                    except Exception as exc:
                        self.get_logger().warning(f"OSNet embedding failed: {exc}")


            detections.append(
                Detection(
                    cx=cx,
                    cy=cy,
                    w=w,
                    h=h,
                    score=score,
                    class_id=class_id,
                    embedding=embedding,
                )
            )

        return detections

    @staticmethod
    def _stamp_to_seconds(msg: Detection2DArray) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9



def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()