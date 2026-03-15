from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    scene_understanding_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("scene_understanding"), "/launch/scene_understanding.launch.py"]
        )
    )

    tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("tracking"), "/launch/tracking.launch.py"]
        )
    )

    return LaunchDescription([
        scene_understanding_launch,
        tracking_launch,
    ])