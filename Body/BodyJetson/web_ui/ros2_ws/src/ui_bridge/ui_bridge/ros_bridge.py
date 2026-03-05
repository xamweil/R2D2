# ui_bridge/ros_bridge.py
import time
import queue
from typing import Any, Dict, Optional, Tuple
import threading 

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from .config import NODE_NAME, ALLOWED_TOPICS, STALE_SEC
from .ros_introspection import import_msg_class, import_srv_class
from .serialize import to_jsonable
from .ros_types import RosCmd

def now_s() -> float:
    return time.time()

class WebUiBridge(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self._subs: Dict[str, Tuple[Any, str]] = {}
        self._state: Dict[str, Dict[str, Any]] = {}

        # MJPEG latest frame store
        self._cam_lock = threading.Lock()
        self._cam_jpeg: Optional[bytes] = None
        self._cam_ts: float = 0.0
        self._cam_sub_alias = "camera_mjpeg"   # internal alias for the subscription
        self._cam_refcount = 0

    def _update_state(self, alias: str, msg: Any) -> None:
        self._state[alias] = {"t": now_s(), "data": to_jsonable(msg)}

    def subscribe_alias(self, alias: str, topic_name: str, msg_type_str: str) -> None:
        if alias in self._subs:
            return
        msg_cls = import_msg_class(msg_type_str)
        sub = self.create_subscription(
            msg_cls, topic_name,
            lambda m, a=alias: self._update_state(a, m),
            10
        )
        self._subs[alias] = (sub, msg_type_str)
        self.get_logger().info(f"Subscribed alias='{alias}' -> {topic_name} [{msg_type_str}]")
    
    def _cam_cb(self, msg: Any) -> None:
        # msg is sensor_msgs/msg/CompressedImage
        # It already contains JPEG bytes in msg.data for format "jpeg"
        with self._cam_lock:
            self._cam_jpeg = bytes(msg.data)
            self._cam_ts = now_s()

    def camera_start(self) -> None:
        self._cam_refcount += 1
        if self._cam_refcount > 1:
            return  # already running

        info = ALLOWED_TOPICS.get("image_raw_compressed")
        if not info:
            raise RuntimeError("image_raw_compressed not in ALLOWED_TOPICS")

        msg_cls = import_msg_class(info["type"])
        topic = info["name"]

        # subscribe (do NOT use the generic subscribe_alias; we want our own callback storing bytes)
        if self._cam_sub_alias in self._subs:
            return

        sub = self.create_subscription(msg_cls, topic, self._cam_cb, 10)
        self._subs[self._cam_sub_alias] = (sub, info["type"])
        self.get_logger().info(f"Camera MJPEG subscribed -> {topic} [{info['type']}]")

    def camera_stop(self) -> None:
        self._cam_refcount = max(0, self._cam_refcount - 1)
        if self._cam_refcount != 0:
            return

        # remove subscription if present
        if self._cam_sub_alias in self._subs:
            sub, _typ = self._subs.pop(self._cam_sub_alias)
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass
            self.get_logger().info("Camera MJPEG unsubscribed")

    def camera_get_latest(self) -> Dict[str, Any]:
        with self._cam_lock:
            if self._cam_jpeg is None:
                return {"ok": False, "error": "no_frame_yet"}
            return {"ok": True, "jpeg": self._cam_jpeg, "t": self._cam_ts}

    def unsubscribe_alias(self, alias: str) -> None:
        if alias not in self._subs:
            return
        sub, _typ = self._subs.pop(alias)
        try:
            self.destroy_subscription(sub)
        except Exception:
            pass
        self.get_logger().info(f"Unsubscribed alias='{alias}'")

    def call_service(self, service_name: str, srv_type_str: str, request_dict: Dict[str, Any], timeout_sec: float):
        srv_cls = import_srv_class(srv_type_str)
        client = self.create_client(srv_cls, service_name)

        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service not available: {service_name}")

        req = srv_cls.Request()
        for k, v in request_dict.items():
            if not hasattr(req, k):
                raise ValueError(f"Request has no field '{k}'")
            setattr(req, k, v)

        fut: Future = client.call_async(req)

        start = now_s()
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if now_s() - start > timeout_sec:
                raise TimeoutError(f"Service call timed out: {service_name}")

        resp = fut.result()

        try:
            self.destroy_client(client)
        except Exception:
            pass

        return resp

    def snapshot_state(self) -> Dict[str, Any]:
        t_now = now_s()
        out: Dict[str, Any] = {"t": t_now, "stale_sec": STALE_SEC, "entries": {}}

        # Always include allowlisted entries even if missing
        for alias in ALLOWED_TOPICS.keys():
            entry = self._state.get(alias)
            if entry is None:
                out["entries"][alias] = {"present": False, "stale": True, "t": None, "data": None}
            else:
                age = t_now - float(entry["t"])
                out["entries"][alias] = {
                    "present": True,
                    "stale": age > STALE_SEC,
                    "age": age,
                    "t": entry["t"],
                    "data": entry["data"],
                }

        # Include dev-added subs too
        for alias, entry in self._state.items():
            if alias in out["entries"]:
                continue
            age = t_now - float(entry["t"])
            out["entries"][alias] = {
                "present": True,
                "stale": age > STALE_SEC,
                "age": age,
                "t": entry["t"],
                "data": entry["data"],
            }

        return out


def ros_thread_main(ros_cmd_q: "queue.Queue[RosCmd]") -> None:
    rclpy.init()
    node = WebUiBridge()

    PRE_SUBSCRIBE_ALIASES = {
        "imu_left_foot",
        "imu_left_leg",
        "imu_right_foot",
        "imu_right_leg",
        "imu_body",
        "camera_info",
        "image_raw_compressed",
    }

    for alias in PRE_SUBSCRIBE_ALIASES:
        info = ALLOWED_TOPICS.get(alias)
        if not info:
            continue
        try:
            node.subscribe_alias(alias, info["name"], info["type"])
        except Exception as e:
            node.get_logger().warning(f"Failed initial subscribe for '{alias}': {e}")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            try:
                cmd = ros_cmd_q.get_nowait()
            except queue.Empty:
                continue

            try:
                if cmd.kind == "subscribe":
                    node.subscribe_alias(cmd.data["alias"], cmd.data["topic"], cmd.data["type"])
                    cmd.reply_q.put({"ok": True})

                elif cmd.kind == "unsubscribe":
                    node.unsubscribe_alias(cmd.data["alias"])
                    cmd.reply_q.put({"ok": True})

                elif cmd.kind == "call_service":
                    resp = node.call_service(
                        cmd.data["service"],
                        cmd.data["type"],
                        cmd.data["request"],
                        timeout_sec=float(cmd.data.get("timeout_sec", 2.0)),
                    )
                    cmd.reply_q.put({"ok": True, "response": to_jsonable(resp)})

                elif cmd.kind == "snapshot":
                    cmd.reply_q.put({"ok": True, "state": node.snapshot_state()})
                
                elif cmd.kind == "camera_start":
                    node.camera_start()
                    cmd.reply_q.put({"ok": True})

                elif cmd.kind == "camera_stop":
                    node.camera_stop()
                    cmd.reply_q.put({"ok": True})

                elif cmd.kind == "camera_get":
                    res = node.camera_get_latest()
                    # IMPORTANT: do not to_jsonable() raw bytes
                    if not res["ok"]:
                        cmd.reply_q.put(res)
                    else:
                        cmd.reply_q.put({"ok": True, "jpeg": res["jpeg"], "t": res["t"]})

                else:
                    cmd.reply_q.put({"ok": False, "error": f"unknown_cmd_kind:{cmd.kind}"})

            except Exception as e:
                cmd.reply_q.put({"ok": False, "error": str(e)})

    finally:
        node.destroy_node()
        rclpy.shutdown()