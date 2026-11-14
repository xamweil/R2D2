import os
from typing import Dict, Tuple

NODE_NAME = os.environ.get("UI_BRIDGE_NODE_NAME", "web_ui_bridge")
HOST = os.environ.get("UI_BRIDGE_HOST", "0.0.0.0")
PORT = int(os.environ.get("UI_BRIDGE_PORT", "8000"))

DEV_UNSAFE = os.environ.get("DEV_UNSAFE", "0").lower() in ("1", "true", "yes")

STATE_HZ = float(os.environ.get("STATE_HZ", "10"))
STATE_PERIOD = 1.0 / max(STATE_HZ, 0.1)
STALE_SEC = float(os.environ.get("STATE_STALE_SEC", "2.0"))

# Allowlist aliases (stable frontend contract)
ALLOWED_TOPICS: Dict[str, Dict[str, str]] = {
    "imu_left_foot": {"name": "/leg_l/imu/foot", "type": "tcp_msg/msg/MPU6500Sample"},
    "imu_left_leg": {"name": "/leg_l/imu/leg", "type": "tcp_msg/msg/MPU6500Sample"},
    "imu_right_foot": {"name": "/leg_r/imu/foot", "type": "tcp_msg/msg/MPU6500Sample"},
    "imu_right_leg": {"name": "/leg_r/imu/leg", "type": "tcp_msg/msg/MPU6500Sample"},
    "imu_body": {"name": "/Body/mpu", "type": "tcp_msg/msg/MPU6500Sample"},

    "camera_info": {"name": "/relay/camera/camera_info", "type": "sensor_msgs/msg/CameraInfo"},
    "image_raw_compressed": {"name": "/relay/camera/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage"},

    "scene_detections": {
    "name": "/scene_understanding/detections",
    "type": "vision_msgs/msg/Detection2DArray",
    },

    "tracking_tracks": {
    "name": "/tracking/tracks",
    "type": "vision_msgs/msg/Detection2DArray",
    },
}

ALLOWED_SERVICES: Dict[str, Dict[str, str]] = {
    "device_command": {"name": "/serial_command", "type": "serial_msg/srv/DeviceCommand"},
}

ALLOWED_PUBLISH_TOPICS: Dict[str, Dict[str, str]] = {
    "motor_command": {"name": "/motor_command", "type": "serial_msg/msg/MotorCommand"},
}

ALLOWED_ACTIONS: Dict[str, Dict[str, str]] = {
    "follow_track": {
        "name": "/follow_track",
        "type": "behavior_msgs/action/FollowTrack",
    },
}

CAMERA_ALIAS = "image_raw_compressed"
CAMERA_FPS = float(os.environ.get("CAMERA_FPS", "3"))  # low fps
CAMERA_PERIOD = 1.0 / max(CAMERA_FPS, 0.5)

# Teleop: frontend joysticks -> sensor_msgs/msg/Joy
TELEOP_TOPIC = os.environ.get("TELEOP_TOPIC", "/teleop")
TELEOP_FRAME_ID = os.environ.get("TELEOP_FRAME_ID", "web_ui")
TELEOP_HZ = float(os.environ.get("TELEOP_HZ", "20"))
TELEOP_TIMEOUT_SEC = float(os.environ.get("TELEOP_TIMEOUT_SEC", "0.25"))

# Frontend stick id (data-stick) -> (x axis, y axis) index in Joy.axes.
# Same layout as joy's game_controller_node: 0 LEFTX, 1 LEFTY, 2 RIGHTX, 3 RIGHTY,
# 4 TRIGGERLEFT, 5 TRIGGERRIGHT (always 0 here).
TELEOP_STICKS: Dict[str, Tuple[int, int]] = {
    "drive": (0, 1),  # left stick: legs / body
    "head": (2, 3),   # right stick: head (frontend locks it to x)
}
TELEOP_NUM_AXES = 6
