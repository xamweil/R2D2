#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from serial_msg.msg import MotorCommand

from tracking.byte_tracker import ByteTracker, Detection


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
        self.declare_parameter("max_time_since_update", 10)

        detection_topic = self.get_parameter("detection_topic").get_parameter_value().string_value
        tracks_topic = self.get_parameter("tracks_topic").get_parameter_value().string_value
        motor_command_topic = self.get_parameter("motor_command_topic").get_parameter_value().string_value
        track_class_ids = list(self.get_parameter("track_class_ids").value)
        high_thresh = float(self.get_parameter("high_thresh").value)
        low_thresh = float(self.get_parameter("low_thresh").value)
        iou_thresh = float(self.get_parameter("iou_thresh").value)
        max_time_since_update = int(self.get_parameter("max_time_since_update").value)

        # Default camera-motion control input u_k = [u_x, u_y]
        self.u_k_x = 0.0
        self.u_k_y = 0.0

        self._last_stamp_sec: float | None = None

        self.tracker = ByteTracker(
            track_class_ids=track_class_ids,
            high_thresh=high_thresh,
            low_thresh=low_thresh,
            iou_thresh=iou_thresh,
            max_time_since_update=max_time_since_update,
        )

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

        self.get_logger().info("tracking started")
        self.get_logger().info(f"  detection_topic: {detection_topic}")
        self.get_logger().info(f"  tracks_topic: {tracks_topic}")
        self.get_logger().info(f"  motor_command_topic: {motor_command_topic}")
        self.get_logger().info(f"  track_class_ids: {track_class_ids}")
        self.get_logger().info(f"  initial u_k: [{self.u_k_x}, {self.u_k_y}]")

    def motor_command_callback(self, msg: MotorCommand) -> None:
        """
        Placeholder for later camera motion handling.

        """
        _ = msg
        self.u_k_x = 0.0
        self.u_k_y = 0.0

    def detections_callback(self, msg: Detection2DArray) -> None:
        stamp_sec = self._stamp_to_seconds(msg)

        if self._last_stamp_sec is None:
            dt = 1.0 / 30.0
        else:
            dt = stamp_sec - self._last_stamp_sec
            if not math.isfinite(dt) or dt <= 0.0:
                dt = 1.0 / 30.0

        self._last_stamp_sec = stamp_sec

        detections = self._convert_ros_detections(msg)

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

    def _convert_ros_detections(self, msg: Detection2DArray) -> list[Detection]:
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

            detections.append(
                Detection(
                    cx=cx,
                    cy=cy,
                    w=w,
                    h=h,
                    score=score,
                    class_id=class_id,
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