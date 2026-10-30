# ui_bridge/state_store.py
import threading
import time
from typing import Any, Dict, NamedTuple, Tuple


class Entry(NamedTuple):
    seq: int
    t: float
    msg: Any


class LatestMessageStore:
    """
    Latest message per alias, shared between the ROS thread and the web layer.
    Messages are stored as received: rclpy builds a new object per take and
    nothing mutates it afterwards, so readers can use it without copying.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._entries: Dict[str, Entry] = {}
        self._seq = 0

    def update(self, alias: str, msg: Any) -> None:
        t = time.time()
        with self._lock:
            self._seq += 1
            self._entries[alias] = Entry(self._seq, t, msg)

    def since(self, cursor: int) -> Tuple[int, Dict[str, Entry]]:
        """Return the current seq and the entries newer than cursor."""
        with self._lock:
            changed = {
                alias: entry
                for alias, entry in self._entries.items()
                if entry.seq > cursor
            }
            return self._seq, changed
