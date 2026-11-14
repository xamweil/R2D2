from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_control',
            executable='teleop_control',
            output='screen'
        ),
    ])
