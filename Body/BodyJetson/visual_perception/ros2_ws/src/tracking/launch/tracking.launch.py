from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    tracking_node = Node(
        package="tracking",
        executable="tracking_node",
        name="tracking",
        output="screen",
        parameters=[
            {
                "detection_topic": "/scene_understanding/detections",
                "tracks_topic": "/tracking/tracks",
                "motor_command_topic": "/motor_command",
                "track_class_ids": [0],
                "image_topic": "/relay/camera/image_raw",
                "reid_engine_path": "/home/ros/models/osnet_x0_25/osnet_x0_25.engine",
                "image_buffer_size": 10,
            }
        ],
    )

    return LaunchDescription([
        tracking_node
    ])