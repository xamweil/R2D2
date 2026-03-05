# ui_bridge/ros_types.py
from dataclasses import dataclass
from typing import Any, Dict
import queue

@dataclass
class RosCmd:
    kind: str
    data: Dict[str, Any]
    reply_q: "queue.Queue[Dict[str, Any]]"