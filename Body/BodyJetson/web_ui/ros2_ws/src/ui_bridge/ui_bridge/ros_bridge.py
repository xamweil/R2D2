# ui_bridge/ros_bridge.py
import queue
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient

from .config import ALLOWED_PUBLISH_TOPICS, ALLOWED_TOPICS, NODE_NAME, STALE_SEC
from .ros_introspection import import_msg_class, import_srv_class, import_action_class
from .ros_types import RosCmd
from .serialize import to_jsonable


def now_s() -> float:
    return time.time()


class WebUiBridge(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self._subs: Dict[str, Tuple[Any, str]] = {}
        self._state: Dict[str, Dict[str, Any]] = {}
        self._pubs: Dict[str, Tuple[Any, str]] = {}

        self._actions: Dict[str, Any] = {}
        self._active_action_goals: Dict[str, Any] = {}

        # MJPEG latest frame store
        self._cam_lock = threading.Lock()
        self._cam_jpeg: Optional[bytes] = None
        self._cam_ts: float = 0.0
        self._cam_sub_alias = "camera_mjpeg"  # internal alias for the subscription
        self._cam_refcount = 0

    def _spin_until(self, done: Callable[[], bool], timeout_sec: float) -> bool:
        """
        Spin this node's executor until done() is true or timeout_sec elapses.
        Keeps subscription callbacks running while waiting, unlike
        wait_for_service()/wait_for_server(), which only sleep.
        Must use the node's own executor: rclpy.spin_once(self) would move the
        node to the global executor and detach it from the main loop's one.
        """
        executor = self.executor
        if executor is None:
            raise RuntimeError("Node is not attached to an executor")

        end = now_s() + timeout_sec
        while rclpy.ok() and not done():
            if now_s() >= end:
                return False
            executor.spin_once(timeout_sec=0.05)
        return done()

    def _update_state(self, alias: str, msg: Any) -> None:
        t = now_s()
        self._state[alias] = {"t": t, "msg": msg}

    def _fill_msg_fields(self, msg: Any, values: Dict[str, Any]) -> None:
        """
        Fill ROS message fields from a plain dict, with explicit coercion for
        serial_msg/msg/MotorCommand field types.
        """

        def as_bool_list(v):
            if not isinstance(v, (list, tuple)):
                raise ValueError("expected a list")
            return [bool(x) for x in v]

        def as_uint8_list(v):
            if not isinstance(v, (list, tuple)):
                raise ValueError("expected a list")
            out = []
            for x in v:
                n = int(x)
                if n < 0 or n > 255:
                    raise ValueError(f"uint8 value out of range: {n}")
                out.append(n)
            return out

        def as_float_list(v):
            if not isinstance(v, (list, tuple)):
                raise ValueError("expected a list")
            return [float(x) for x in v]

        for key, value in values.items():
            if not hasattr(msg, key):
                raise ValueError(f"Message has no field '{key}'")

            # Explicit coercion for MotorCommand fields
            if key in ("ids", "velocity"):
                coerced = as_uint8_list(value)
            elif key in ("enable", "direction", "angle_set", "velocity_set"):
                coerced = as_bool_list(value)
            elif key == "angle":
                coerced = as_float_list(value)
            else:
                coerced = value

            setattr(msg, key, coerced)

    def publish_alias(
        self, alias: str, topic_name: str, msg_type_str: str, msg_dict: Dict[str, Any]
    ) -> None:
        if alias not in self._pubs:
            msg_cls = import_msg_class(msg_type_str)
            pub = self.create_publisher(msg_cls, topic_name, 10)
            self._pubs[alias] = (pub, msg_type_str)
        else:
            pub, _ = self._pubs[alias]
            msg_cls = import_msg_class(msg_type_str)

        msg = msg_cls()
        self._fill_msg_fields(msg, msg_dict)
        pub.publish(msg)

    def send_action_goal(self, alias: str, action_name: str, action_type_str: str,
        goal_dict: Dict[str, Any], timeout_sec: float = 2.0):

        if alias not in self._actions:
            action_cls = import_action_class(action_type_str)
            client = ActionClient(self, action_cls, action_name)
            self._actions[alias] = (client, action_type_str)
        else:
            client, _ = self._actions[alias]
            action_cls = import_action_class(action_type_str)

        if not self._spin_until(client.server_is_ready, timeout_sec):
            raise RuntimeError(f"Action server not available: {action_name}")

        goal_msg = action_cls.Goal()
        self._fill_msg_fields(goal_msg, goal_dict)

        send_future = client.send_goal_async(goal_msg)

        if not self._spin_until(send_future.done, timeout_sec):
            raise TimeoutError(f"Action goal send timed out: {action_name}")

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Action goal rejected: {action_name}")

        self._active_action_goals[alias] = goal_handle
        return {"accepted": True}

    def cancel_action_goal(self, alias: str, timeout_sec: float = 2.0):
        goal_handle = self._active_action_goals.get(alias)
        if goal_handle is None:
            return {"ok": True, "message": "no_active_goal"}

        cancel_future = goal_handle.cancel_goal_async()

        if not self._spin_until(cancel_future.done, timeout_sec):
            raise TimeoutError(f"Cancel action goal timed out for alias: {alias}")

        self._active_action_goals.pop(alias, None)
        return {"ok": True}

    def subscribe_alias(self, alias: str, topic_name: str, msg_type_str: str) -> None:
        if alias in self._subs:
            return
        msg_cls = import_msg_class(msg_type_str)
        sub = self.create_subscription(
            msg_cls, topic_name, lambda m, a=alias: self._update_state(a, m), 10
        )
        self._subs[alias] = (sub, msg_type_str)
        self.get_logger().info(
            f"Subscribed alias='{alias}' -> {topic_name} [{msg_type_str}]"
        )

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
            sub, _ = self._subs.pop(self._cam_sub_alias)
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
        sub, _ = self._subs.pop(alias)
        try:
            self.destroy_subscription(sub)
        except Exception:
            pass
        self.get_logger().info(f"Unsubscribed alias='{alias}'")

    # TODO: make this async(?), handle errors
    def call_service(
        self,
        service_name: str,
        srv_type_str: str,
        request_dict: Dict[str, Any],
        timeout_sec: float,
    ):
        srv_cls = import_srv_class(srv_type_str)
        client = self.create_client(srv_cls, service_name)

        if not self._spin_until(client.service_is_ready, timeout_sec):
            raise RuntimeError(f"Service not available: {service_name}")

        req = srv_cls.Request()
        for k, v in request_dict.items():
            if not hasattr(req, k):
                raise ValueError(f"Request has no field '{k}'")
            setattr(req, k, v)

        fut: Future = client.call_async(req)

        if not self._spin_until(fut.done, timeout_sec):
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
                out["entries"][alias] = {
                    "present": False,
                    "stale": True,
                    "t": None,
                    "data": None,
                }
            else:
                age = t_now - float(entry["t"])
                out["entries"][alias] = {
                    "present": True,
                    "stale": age > STALE_SEC,
                    "age": age,
                    "t": entry["t"],
                    "data": to_jsonable(entry["msg"]),
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
                "data": to_jsonable(entry["msg"]),
            }

        return out


def _handle_cmd(node: WebUiBridge, cmd: RosCmd) -> Dict[str, Any]:
    if cmd.kind == "subscribe":
        node.subscribe_alias(cmd.data["alias"], cmd.data["topic"], cmd.data["type"])
        return {"ok": True}

    elif cmd.kind == "unsubscribe":
        node.unsubscribe_alias(cmd.data["alias"])
        return {"ok": True}

    elif cmd.kind == "call_service":
        resp = node.call_service(
            cmd.data["service"],
            cmd.data["type"],
            cmd.data["request"],
            timeout_sec=float(cmd.data.get("timeout_sec", 2.0)),
        )
        return {"ok": True, "response": to_jsonable(resp)}

    elif cmd.kind == "snapshot":
        return {"ok": True, "state": node.snapshot_state()}

    elif cmd.kind == "camera_start":
        node.camera_start()
        return {"ok": True}

    elif cmd.kind == "camera_stop":
        node.camera_stop()
        return {"ok": True}

    elif cmd.kind == "camera_get":
        res = node.camera_get_latest()
        # IMPORTANT: do not to_jsonable() raw bytes
        if not res["ok"]:
            return res
        return {"ok": True, "jpeg": res["jpeg"], "t": res["t"]}

    elif cmd.kind == "publish":
        node.publish_alias(
                cmd.data["alias"],
                cmd.data["topic"],
                cmd.data["type"],
                cmd.data["msg"]
        )
        return {"ok": True}

    elif cmd.kind == "send_action_goal":
        result = node.send_action_goal(
            cmd.data["alias"],
            cmd.data["action"],
            cmd.data["type"],
            cmd.data["goal"],
            float(cmd.data.get("timeout_sec", 2.0))
        )
        return {"ok": True, "result": result}

    elif cmd.kind == "cancel_action_goal":
        result = node.cancel_action_goal(
            cmd.data["alias"],
            float(cmd.data.get("timeout_sec", 2.0))
        )
        return {"ok": True, "result": result}

    else:
        node.get_logger().warning(f"Unknown command kind: '{cmd.kind}'")
        return {"ok": False, "error": f"unknown_cmd_kind:{cmd.kind}"}


def ros_thread_main(ros_cmd_q: queue.Queue[RosCmd]) -> None:
    rclpy.init()
    node = WebUiBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    PRE_SUBSCRIBE_ALIASES = {
        "imu_left_foot",
        "imu_left_leg",
        "imu_right_foot",
        "imu_right_leg",
        "imu_body",
        "camera_info",
        "image_raw_compressed",
        "scene_detections",
        "tracking_tracks",
    }

    for alias in PRE_SUBSCRIBE_ALIASES:
        info = ALLOWED_TOPICS.get(alias)
        if not info:
            node.get_logger().warning(f"Pre subscribe alias {alias} not in ALLOWED_TIOICS")
            continue
        try:
            node.subscribe_alias(alias, info["name"], info["type"])
        except Exception as e:
            node.get_logger().warning(f"Failed initial subscribe for '{alias}': {e}")

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

            while True:
                try:
                    cmd = ros_cmd_q.get_nowait()
                except queue.Empty:
                    break
                try:
                    reply = _handle_cmd(node, cmd)
                except Exception as e:
                    node.get_logger().error(f"Command '{cmd.kind}' failed: {e}")
                    reply = {"ok": False, "error": str(e)}
                cmd.reply_q.put(reply)

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
