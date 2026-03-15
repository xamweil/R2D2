#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from serial_msg.msg import MotorCommand


class TrackingNode(Node):
    def __init__(self) -> None:
        super().__init__("tracking")

        self.declare_parameter("detection_topic", "/scene_understanding/detections")
        self.declare_parameter("tracks_topic", "/tracking/tracks")
        self.declare_parameter("motor_command_topic", "/motor_command")
        self.declare_parameter("track_class_ids", [0])

        detection_topic = self.get_parameter("detection_topic").get_parameter_value().string_value
        tracks_topic = self.get_parameter("tracks_topic").get_parameter_value().string_value
        motor_command_topic = self.get_parameter("motor_command_topic").get_parameter_value().string_value
        track_class_ids = list(self.get_parameter("track_class_ids").value)

        # Default camera motion control input u_k = [u_x, u_y]
        # For now: no camera motion compensation.
        self.u_k = [0.0, 0.0]

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
        self.get_logger().info(f"  initial u_k: {self.u_k}")

    def motor_command_callback(self, msg: MotorCommand) -> None:
        """
        Placeholder for later camera motion handling.

        Later this callback will:
        - filter relevant motor IDs
        - interpret angle/velocity modes
        - convert motor commands to image-plane control input u_k
        - reset timed angle commands when their expected motion window ends
        """
        _ = msg

    def detections_callback(self, msg: Detection2DArray) -> None:
        """
        Placeholder tracking callback.

        For now we simply republish an empty tracks message with the same header.
        Next step:
        - convert detections to internal arrays
        - call ByteTracker
        - publish tracked detections with stable IDs
        """
        tracks_msg = Detection2DArray()
        tracks_msg.header = msg.header
        self._tracks_pub.publish(tracks_msg)


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