# ui_bridge/teleop.py
import math
import threading
import time
from typing import Dict, List, Tuple

from .config import TELEOP_NUM_AXES, TELEOP_STICKS, TELEOP_TIMEOUT_SEC


def _axis(v) -> float:
    v = float(v)
    if not math.isfinite(v):
        raise ValueError("axis value must be finite")
    return max(-1.0, min(1.0, v))


class TeleopState:
    """
    Latest stick deflection per frontend stick id.
    Written by the web (asyncio) thread, read by the ROS timer.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sticks: Dict[str, Tuple[float, float, float]] = {}  # id -> (x, y, monotonic t)

    def set(self, stick_id: str, x, y) -> None:
        if stick_id not in TELEOP_STICKS:
            raise KeyError(stick_id)
        sample = (_axis(x), _axis(y), time.monotonic())
        with self._lock:
            self._sticks[stick_id] = sample

    def stop(self) -> None:
        with self._lock:
            self._sticks.clear()

    def axes(self) -> List[float]:
        """
        sensor_msgs/Joy axes in game_controller_node layout.
        A stick without an update for TELEOP_TIMEOUT_SEC counts as centered,
        so a dropped connection stops the robot.
        """
        now = time.monotonic()
        axes = [0.0] * TELEOP_NUM_AXES
        with self._lock:
            for stick_id, (x, y, t) in self._sticks.items():
                if now - t > TELEOP_TIMEOUT_SEC:
                    continue
                ix, iy = TELEOP_STICKS[stick_id]
                axes[ix] = -x  # Joy convention: left is positive (frontend: right is positive)
                axes[iy] = y   # up is positive in both
        return axes
