from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            # parameters=[{'device_name': 'PS4 Controller'}]
        ),
        Node(
            package='motor_control',
            executable='motor_control',
            output='screen'
        ),
    ])
