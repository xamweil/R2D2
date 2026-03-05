import json
import asyncio
import queue
from typing import Any, Dict, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, StreamingResponse
import os

from .config import DEV_UNSAFE, STATE_PERIOD, STATE_HZ, STALE_SEC, ALLOWED_TOPICS, ALLOWED_SERVICES, NODE_NAME, CAMERA_PERIOD
from .ros_types import RosCmd


class AllowedCall(BaseModel):
    alias: str
    request: Dict[str, Any] = {}
    timeout_sec: float = 2.0


class GenericServiceCall(BaseModel):
    service: str
    type: str
    request: Dict[str, Any] = {}
    timeout_sec: float = 2.0


class GenericSubscribe(BaseModel):
    alias: str
    topic: str
    type: str


def create_app(ros_cmd_q: "queue.Queue[RosCmd]") -> FastAPI:
    app = FastAPI(title="web_ui")

    frontend_dir = os.environ.get("WEB_UI_FRONTEND_DIR", "/home/ros/frontend")

    if os.path.isdir(frontend_dir):
        app.mount("/static", StaticFiles(directory=frontend_dir), name="static")

        @app.get("/")
        def index():
            return FileResponse(os.path.join(frontend_dir, "index.html"))

    ws_clients: Set[WebSocket] = set()
    ws_lock = asyncio.Lock()

    @app.get("/health")
    def health():
        return {"ok": True, "dev_unsafe": DEV_UNSAFE, "node": NODE_NAME, "state_hz": STATE_HZ}

    @app.post("/call/allowed")
    def call_allowed(body: AllowedCall):
        if body.alias not in ALLOWED_SERVICES:
            return {"ok": False, "error": "unknown_allowed_service_alias"}

        info = ALLOWED_SERVICES[body.alias]
        reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)

        ros_cmd_q.put(RosCmd(
            kind="call_service",
            data={"service": info["name"], "type": info["type"], "request": body.request, "timeout_sec": body.timeout_sec},
            reply_q=reply_q,
        ))
        return reply_q.get(timeout=body.timeout_sec + 1.0)

    @app.get("/mjpeg")
    async def mjpeg():
        """
        Streams camera frames as MJPEG. On-demand subscription: starts when a client connects,
        stops when the client disconnects. Throttled heavily (CAMERA_FPS).
        """
        # start camera subscription
        start_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
        ros_cmd_q.put(RosCmd(kind="camera_start", data={}, reply_q=start_q))
        _ = start_q.get(timeout=2.0)

        boundary = "frame"

        async def gen():
            last_sent_t = 0.0
            try:
                while True:
                    await asyncio.sleep(CAMERA_PERIOD)

                    reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
                    try:
                        ros_cmd_q.put_nowait(RosCmd(kind="camera_get", data={}, reply_q=reply_q))
                    except queue.Full:
                        continue

                    try:
                        res = reply_q.get(timeout=1.0)
                    except queue.Empty:
                        continue

                    if not res.get("ok"):
                        continue

                    t = float(res.get("t", 0.0))
                    if t <= last_sent_t:
                        continue
                    last_sent_t = t

                    jpeg: bytes = res["jpeg"]
                    yield (
                        f"--{boundary}\r\n"
                        "Content-Type: image/jpeg\r\n"
                        f"Content-Length: {len(jpeg)}\r\n\r\n"
                    ).encode("utf-8") + jpeg + b"\r\n"
            finally:
                stop_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
                ros_cmd_q.put(RosCmd(kind="camera_stop", data={}, reply_q=stop_q))
                try:
                    stop_q.get(timeout=2.0)
                except queue.Empty:
                    pass

        return StreamingResponse(gen(), media_type=f"multipart/x-mixed-replace; boundary={boundary}")

    @app.post("/call")
    def call_generic(body: GenericServiceCall):
        if not DEV_UNSAFE:
            return {"ok": False, "error": "generic_service_calls_disabled_set_DEV_UNSAFE=1"}

        reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
        ros_cmd_q.put(RosCmd(
            kind="call_service",
            data={"service": body.service, "type": body.type, "request": body.request, "timeout_sec": body.timeout_sec},
            reply_q=reply_q,
        ))
        return reply_q.get(timeout=body.timeout_sec + 1.0)

    @app.post("/subscribe")
    def subscribe(body: GenericSubscribe):
        # Allowlist alias is always permitted; otherwise requires DEV_UNSAFE
        if body.alias in ALLOWED_TOPICS:
            info = ALLOWED_TOPICS[body.alias]
            topic = info["name"]
            typ = info["type"]
        else:
            if not DEV_UNSAFE:
                return {"ok": False, "error": "generic_subscribe_disabled_set_DEV_UNSAFE=1"}
            topic = body.topic
            typ = body.type

        reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
        ros_cmd_q.put(RosCmd(kind="subscribe", data={"alias": body.alias, "topic": topic, "type": typ}, reply_q=reply_q))
        return reply_q.get(timeout=2.0)

    @app.post("/unsubscribe")
    def unsubscribe(body: Dict[str, str]):
        alias = body.get("alias", "")
        reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
        ros_cmd_q.put(RosCmd(kind="unsubscribe", data={"alias": alias}, reply_q=reply_q))
        return reply_q.get(timeout=2.0)

    async def broadcast_loop():
        while True:
            await asyncio.sleep(STATE_PERIOD)

            reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
            try:
                ros_cmd_q.put_nowait(RosCmd(kind="snapshot", data={}, reply_q=reply_q))
            except queue.Full:
                continue

            try:
                resp = reply_q.get(timeout=1.0)
            except queue.Empty:
                continue

            if not resp.get("ok"):
                payload = {"type": "state_error", "payload": {"error": resp.get("error", "unknown")}}
            else:
                payload = {"type": "robot_state", "payload": resp["state"]}

            async with ws_lock:
                dead = set()
                for ws in ws_clients:
                    try:
                        await ws.send_json(payload)
                    except Exception:
                        dead.add(ws)
                for ws in dead:
                    ws_clients.discard(ws)

    @app.on_event("startup")
    async def on_startup():
        asyncio.create_task(broadcast_loop())

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        async with ws_lock:
            ws_clients.add(ws)

        await ws.send_json({
            "type": "hello",
            "payload": {
                "dev_unsafe": DEV_UNSAFE,
                "allowed_topics": ALLOWED_TOPICS,
                "allowed_services": ALLOWED_SERVICES,
                "state_hz": STATE_HZ,
                "stale_sec": STALE_SEC,
            }
        })

        try:
            while True:
                raw = await ws.receive_text()
                try:
                    data = json.loads(raw)
                except Exception:
                    await ws.send_json({"type": "error", "payload": {"error": "invalid_json"}})
                    continue

                mtype = data.get("type", "")
                if mtype == "call_allowed":
                    alias = data.get("alias", "")
                    req = data.get("request", {}) or {}
                    timeout_sec = float(data.get("timeout_sec", 2.0))

                    if alias not in ALLOWED_SERVICES:
                        await ws.send_json({"type": "call_result", "payload": {"ok": False, "error": "unknown_alias"}})
                        continue

                    info = ALLOWED_SERVICES[alias]
                    reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
                    ros_cmd_q.put(RosCmd(
                        kind="call_service",
                        data={"service": info["name"], "type": info["type"], "request": req, "timeout_sec": timeout_sec},
                        reply_q=reply_q
                    ))
                    try:
                        resp = reply_q.get(timeout=timeout_sec + 1.0)
                    except queue.Empty:
                        resp = {"ok": False, "error": "timeout_waiting_for_ros"}
                    await ws.send_json({"type": "call_result", "payload": resp})

                elif mtype == "call":
                    if not DEV_UNSAFE:
                        await ws.send_json({"type": "call_result", "payload": {"ok": False, "error": "DEV_UNSAFE_required"}})
                        continue

                    srv = data.get("service", "")
                    typ = data.get("type_name", "") or data.get("type", "")
                    req = data.get("request", {}) or {}
                    timeout_sec = float(data.get("timeout_sec", 2.0))

                    reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
                    ros_cmd_q.put(RosCmd(
                        kind="call_service",
                        data={"service": srv, "type": typ, "request": req, "timeout_sec": timeout_sec},
                        reply_q=reply_q
                    ))
                    try:
                        resp = reply_q.get(timeout=timeout_sec + 1.0)
                    except queue.Empty:
                        resp = {"ok": False, "error": "timeout_waiting_for_ros"}
                    await ws.send_json({"type": "call_result", "payload": resp})

                elif mtype == "subscribe":
                    alias = data.get("alias", "")
                    if alias in ALLOWED_TOPICS:
                        info = ALLOWED_TOPICS[alias]
                        topic = info["name"]
                        typ = info["type"]
                    else:
                        if not DEV_UNSAFE:
                            await ws.send_json({"type": "subscribe_result", "payload": {"ok": False, "error": "DEV_UNSAFE_required"}})
                            continue
                        topic = data.get("topic", "")
                        typ = data.get("type_name", "") or data.get("type", "")

                    reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
                    ros_cmd_q.put(RosCmd(kind="subscribe", data={"alias": alias, "topic": topic, "type": typ}, reply_q=reply_q))
                    try:
                        resp = reply_q.get(timeout=2.0)
                    except queue.Empty:
                        resp = {"ok": False, "error": "timeout_waiting_for_ros"}
                    await ws.send_json({"type": "subscribe_result", "payload": resp})

                elif mtype == "unsubscribe":
                    alias = data.get("alias", "")
                    reply_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
                    ros_cmd_q.put(RosCmd(kind="unsubscribe", data={"alias": alias}, reply_q=reply_q))
                    try:
                        resp = reply_q.get(timeout=2.0)
                    except queue.Empty:
                        resp = {"ok": False, "error": "timeout_waiting_for_ros"}
                    await ws.send_json({"type": "unsubscribe_result", "payload": resp})

                else:
                    await ws.send_json({"type": "error", "payload": {"error": f"unknown_message_type:{mtype}"}})

        except WebSocketDisconnect:
            pass
        finally:
            async with ws_lock:
                ws_clients.discard(ws)

    return app