from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    scene_understanding_node = Node(
        package="scene_understanding",
        executable="scene_understanding_node",
        name="scene_understanding",
        output="screen",

        parameters=[
            {
                "image_topic": "/relay/camera/image_raw",
                "detection_topic": "/scene_understanding/detections",
                "engine_path": "/home/ros/models/yolov8n/yolov8n.engine",
            }
        ],
    )

    return LaunchDescription([
        scene_understanding_node
    ])