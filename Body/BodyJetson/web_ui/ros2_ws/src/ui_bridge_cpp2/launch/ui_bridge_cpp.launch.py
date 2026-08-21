from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    doc_root = LaunchConfiguration("doc_root")
    mjpeg_fps = LaunchConfiguration("mjpeg_fps")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="9090",
            description="HTTP listen port",
        ),
        DeclareLaunchArgument(
            "doc_root",
            default_value="/home/ros/frontend",
            description="Path to the frontend static files",
        ),
        DeclareLaunchArgument(
            "mjpeg_fps",
            default_value="3",
            description="MJPEG stream frame rate",
        ),

        Node(
            package="ui_bridge_cpp",
            executable="ui_bridge_cpp_node",
            name="ui_bridge_cpp",
            output="screen",
            parameters=[{
                "port": port,
                "doc_root": doc_root,
                "mjpeg_fps": mjpeg_fps,
            }],
        ),
    ])
