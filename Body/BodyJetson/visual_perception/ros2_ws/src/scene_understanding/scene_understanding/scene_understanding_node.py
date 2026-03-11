from __future__ import annotations

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

from cv_bridge import CvBridge

from scene_understanding.image_preprocessor import ImagePreprocessor
from scene_understanding.tensor_rt_engine import TensorRTEngine
from scene_understanding.yolo_decoder import YOLODecoder


class SceneUnderstandingNode(Node):

    def __init__(self):

        super().__init__("scene_understanding")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detection_topic", "/scene_understanding/detections")
        self.declare_parameter("engine_path", "/home/ros/models/yolov8n/yolov8n.engine")

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        detection_topic = self.get_parameter("detection_topic").get_parameter_value().string_value
        engine_path = self.get_parameter("engine_path").get_parameter_value().string_value

        self.get_logger().info("scene_understanding started")
        self.get_logger().info(f"  image_topic: {image_topic}")
        self.get_logger().info(f"  detection_topic: {detection_topic}")
        self.get_logger().info(f"  engine_path: {engine_path}")

        self.bridge = CvBridge()

        self.preprocessor = ImagePreprocessor()
        self.engine = TensorRTEngine(engine_path)
        self.decoder = YOLODecoder()

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Detection2DArray,
            detection_topic,
            10
        )

    def image_callback(self, msg: Image):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        tensor = self.preprocessor.preprocess(image)

        outputs = self.engine.infer(tensor)

        boxes, scores, classes = self.decoder.decode(outputs["output0"])

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for box, score, cls in zip(boxes, scores, classes):

            detection = Detection2D()

            bbox = BoundingBox2D()

            x1, y1, x2, y2 = box

            bbox.center.position.x = float((x1 + x2) / 2.0)
            bbox.center.position.y = float((y1 + y2) / 2.0)

            bbox.size_x = float(x2 - x1)
            bbox.size_y = float(y2 - y1)

            detection.bbox = bbox

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(cls))
            hypothesis.hypothesis.score = float(score)

            detection.results.append(hypothesis)

            detections_msg.detections.append(detection)

        self.publisher.publish(detections_msg)


def main():

    rclpy.init()

    node = SceneUnderstandingNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()