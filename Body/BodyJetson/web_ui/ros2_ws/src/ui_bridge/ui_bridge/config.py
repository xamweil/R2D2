import os
from typing import Dict

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

    "camera_info": {"name": "/camera/camera_info", "type": "sensor_msgs/msg/CameraInfo"},
    "image_raw_compressed": {"name": "/camera/image_raw/compressed", "type": "sensor_msgs/msg/CompressedImage"},
}

ALLOWED_SERVICES: Dict[str, Dict[str, str]] = {
    "device_command": {"name": "/serial_command", "type": "serial_msg/srv/DeviceCommand"},
}

ALLOWED_PUBLISH_TOPICS: Dict[str, Dict[str, str]] = {
    "motor_command": {"name": "/motor_command", "type": "serial_msg/msg/MotorCommand"},
}

CAMERA_ALIAS = "image_raw_compressed"
CAMERA_FPS = float(os.environ.get("CAMERA_FPS", "3"))  # low fps
CAMERA_PERIOD = 1.0 / max(CAMERA_FPS, 0.5)