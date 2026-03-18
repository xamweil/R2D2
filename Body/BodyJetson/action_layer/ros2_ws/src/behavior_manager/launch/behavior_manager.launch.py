from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    behavior_manager_node = Node(
        package="behavior_manager",
        executable="behavior_manager_node",
        name="behavior_manager",
        output="screen",
        parameters=[
            {
                "tracking_topic": "/tracking/tracks",
                "motor_command_topic": "/motor_command",
                "status_topic": "/behavior_manager/status",
                "update_rate_hz": 10.0,
                "status_rate_hz": 2.0,

                "follow_track_head_motor_id": 1,
                "follow_track_max_velocity": 80,
                "follow_track_center_deadband": 0.08,
                "follow_track_bbox_is_normalized": True,
                "follow_track_image_width_px": 640,
                "follow_track_invert_head_direction": True,
            }
        ],
    )

    return LaunchDescription([
        behavior_manager_node
    ])