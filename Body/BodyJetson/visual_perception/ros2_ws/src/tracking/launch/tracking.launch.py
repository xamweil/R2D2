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
            }
        ],
    )

    return LaunchDescription([
        tracking_node
    ])