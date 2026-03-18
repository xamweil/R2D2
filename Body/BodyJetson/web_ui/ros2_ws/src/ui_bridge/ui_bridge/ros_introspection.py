import importlib

def import_msg_class(type_str: str):
    """
    "sensor_msgs/msg/Image" or "tcp_msg/msg/MPU6500Sample"
    """
    try:
        pkg, kind, name = type_str.split("/")
        if kind != "msg":
            raise ValueError("not a msg type")
        mod = importlib.import_module(f"{pkg}.msg")
        return getattr(mod, name)
    except Exception as e:
        raise ValueError(f"Failed to import message type '{type_str}': {e}") from e

def import_srv_class(type_str: str):
    """
    "serial_msg/srv/DeviceCommand"
    """
    try:
        pkg, kind, name = type_str.split("/")
        if kind != "srv":
            raise ValueError("not a srv type")
        mod = importlib.import_module(f"{pkg}.srv")
        return getattr(mod, name)
    except Exception as e:
        raise ValueError(f"Failed to import service type '{type_str}': {e}") from e

def import_action_class(type_str: str):
    pkg, iface = type_str.split("/", 1)
    submodule, cls_name = iface.split("/", 1)   # action/FollowTrack
    module = __import__(f"{pkg}.{submodule}", fromlist=[cls_name])
    return getattr(module, cls_name)