from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_compressed_topic = LaunchConfiguration("input_compressed_topic")
    input_camera_info_topic = LaunchConfiguration("input_camera_info_topic")
    output_compressed_topic = LaunchConfiguration("output_compressed_topic")
    output_raw_topic = LaunchConfiguration("output_raw_topic")
    output_camera_info_topic = LaunchConfiguration("output_camera_info_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_compressed_topic",
            default_value="/camera/image_raw/compressed"
        ),
        DeclareLaunchArgument(
            "input_camera_info_topic",
            default_value="/camera/camera_info"
        ),
        DeclareLaunchArgument(
            "output_compressed_topic",
            default_value="/relay/camera/image_raw/compressed"
        ),
        DeclareLaunchArgument(
            "output_raw_topic",
            default_value="/relay/camera/image_raw"
        ),
        DeclareLaunchArgument(
            "output_camera_info_topic",
            default_value="/relay/camera/camera_info"
        ),

        Node(
            package="camera_relay",
            executable="camera_stream_relay_node",
            name="camera_stream_relay",
            output="screen",
            parameters=[{
                "input_topic": input_compressed_topic,
                "output_compressed_topic": output_compressed_topic,
                "output_raw_topic": output_raw_topic,
            }]
        ),

        Node(
            package="camera_relay",
            executable="camera_info_relay_node",
            name="camera_info_relay",
            output="screen",
            parameters=[{
                "input_topic": input_camera_info_topic,
                "output_topic": output_camera_info_topic,
            }]
        ),
    ])