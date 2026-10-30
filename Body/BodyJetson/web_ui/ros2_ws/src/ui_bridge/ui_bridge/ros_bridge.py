# ui_bridge/ros_bridge.py
import queue
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

import rclpy
from rclpy.executors import ShutdownException, SingleThreadedExecutor, TimeoutException
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient

from .config import ALLOWED_PUBLISH_TOPICS, ALLOWED_TOPICS, INTAKE_PERIOD, NODE_NAME
from .ros_introspection import import_msg_class, import_srv_class, import_action_class
from .ros_types import RosCmd
from .state_store import LatestMessageStore

# DDS keeps only the newest unread message, which is all one intake tick takes
SUB_QOS_DEPTH = 1


def now_s() -> float:
    return time.time()


def _fill_msg_fields(msg: Any, values: Dict[str, Any]) -> None:
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


class WebUiBridge(Node):
    def __init__(self, store: LatestMessageStore) -> None:
        super().__init__(NODE_NAME)
        self._store = store
        self._subs: Dict[str, Tuple[Any, str]] = {}
        self._pubs: Dict[str, Tuple[Any, str]] = {}

        # MJPEG latest frame store
        self._cam_lock = threading.Lock()
        self._cam_jpeg: Optional[bytes] = None
        self._cam_ts: float = 0.0
        self._cam_sub_alias = "camera_mjpeg"  # internal alias for the subscription
        self._cam_refcount = 0

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
        _fill_msg_fields(msg, msg_dict)
        pub.publish(msg)

    def subscribe_alias(self, alias: str, topic_name: str, msg_type_str: str) -> None:
        if alias in self._subs:
            return
        msg_cls = import_msg_class(msg_type_str)
        sub = self.create_subscription(
            msg_cls, topic_name, lambda m, a=alias: self._store.update(a, m), SUB_QOS_DEPTH
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

        sub = self.create_subscription(msg_cls, topic, self._cam_cb, SUB_QOS_DEPTH)
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


class WebUiCalls(Node):
    """
    Service and action calls, which block until the remote side answers. They
    get their own node and thread so that a slow call holds up no other command,
    and waiting for its reply spins none of the subscriptions.
    """

    def __init__(self) -> None:
        super().__init__(f"{NODE_NAME}_calls")
        self._actions: Dict[str, Any] = {}
        self._active_action_goals: Dict[str, Any] = {}

    def _spin_until(self, done: Callable[[], bool], timeout_sec: float) -> bool:
        """
        Spin this node's executor until done() is true or timeout_sec elapses.
        Must use the node's own executor: rclpy.spin_once(self) would move the
        node to the global executor and detach it from the calls thread's one.
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
        _fill_msg_fields(goal_msg, goal_dict)

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

    def call_service(
        self,
        service_name: str,
        srv_type_str: str,
        request_dict: Dict[str, Any],
        timeout_sec: float,
    ):
        srv_cls = import_srv_class(srv_type_str)
        client = self.create_client(srv_cls, service_name)

        try:
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

            return fut.result()
        finally:
            try:
                self.destroy_client(client)
            except Exception:
                pass


def _handle_cmd(node: WebUiBridge, cmd: RosCmd) -> Dict[str, Any]:
    if cmd.kind == "subscribe":
        node.subscribe_alias(cmd.data["alias"], cmd.data["topic"], cmd.data["type"])
        return {"ok": True}

    elif cmd.kind == "unsubscribe":
        node.unsubscribe_alias(cmd.data["alias"])
        return {"ok": True}

    elif cmd.kind == "camera_start":
        node.camera_start()
        return {"ok": True}

    elif cmd.kind == "camera_stop":
        node.camera_stop()
        return {"ok": True}

    elif cmd.kind == "camera_get":
        res = node.camera_get_latest()
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

    else:
        node.get_logger().warning(f"Unknown command kind: '{cmd.kind}'")
        return {"ok": False, "error": f"unknown_cmd_kind:{cmd.kind}"}


def _handle_call(node: WebUiCalls, cmd: RosCmd) -> Dict[str, Any]:
    if cmd.kind == "call_service":
        resp = node.call_service(
            cmd.data["service"],
            cmd.data["type"],
            cmd.data["request"],
            timeout_sec=float(cmd.data.get("timeout_sec", 2.0)),
        )
        return {"ok": True, "response": resp}

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


def _run_cmd(handle: Callable[[Any, RosCmd], Dict[str, Any]], node: Node, cmd: RosCmd) -> None:
    try:
        reply = handle(node, cmd)
    except Exception as e:
        node.get_logger().error(f"Command '{cmd.kind}' failed: {e}")
        reply = {"ok": False, "error": str(e)}
    cmd.reply_q.put(reply)


def _take_ready(executor: SingleThreadedExecutor) -> None:
    """
    Run every callback that is ready now. A zero timeout makes one pass over the
    wait set, so this ends even if new messages keep arriving.
    """
    while True:
        try:
            handler, _, _ = executor.wait_for_ready_callbacks(timeout_sec=0)
        except (ShutdownException, TimeoutException):
            return
        handler()
        handler.result()


def calls_thread_main(ros_call_q: queue.Queue[RosCmd]) -> None:
    node = WebUiCalls()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            try:
                cmd = ros_call_q.get(timeout=1.0)
            except queue.Empty:
                continue
            _run_cmd(_handle_call, node, cmd)

    finally:
        executor.shutdown()
        node.destroy_node()


def ros_thread_main(
    ros_cmd_q: queue.Queue[RosCmd], ros_call_q: queue.Queue[RosCmd], store: LatestMessageStore
) -> None:
    rclpy.init()
    threading.Thread(target=calls_thread_main, args=(ros_call_q,), daemon=True).start()

    node = WebUiBridge(store)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    for alias, info in ALLOWED_TOPICS.items():
        if not info.get("in_state", True):
            continue
        try:
            node.subscribe_alias(alias, info["name"], info["type"])
        except Exception as e:
            node.get_logger().warning(f"Failed initial subscribe for '{alias}': {e}")

    try:
        next_intake = time.monotonic()
        while rclpy.ok():
            wait = next_intake - time.monotonic()
            if wait <= 0:
                _take_ready(executor)
                next_intake = time.monotonic() + INTAKE_PERIOD
                continue

            # Blocks, so a command runs as soon as it arrives
            try:
                cmd = ros_cmd_q.get(timeout=wait)
            except queue.Empty:
                continue
            _run_cmd(_handle_cmd, node, cmd)

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
