#!/usr/bin/env python3
"""
ROS2 bridge for one ESP32 running two MPU‑6500 sensors + motor / buttons
-----------------------------------------------------------------------
Publishes
    imu/leg  (tcp_msg/MPU6500Sample)   # CID 0x01
    imu/foot (tcp_msg/MPU6500Sample)   # CID 0x02

Provides
    cmd      (tcp_msg/XiaoCmd)         # generic command service

Parameters
    ~ip      (string, default "192.168.66.10")
    ~port    (int,    default 5010)
"""

import struct
import rclpy
from rclpy.node import Node

from tcp_msg.msg import MPU6500Sample
from tcp_msg.srv import XiaoCmd
from xiao_bridge.XiaoESP32C3 import XiaoESP32C3

from xiao_bridge.Transport import (
    Transport,
    CID_SENSOR_LEG,
    CID_SENSOR_FOOT,
)

class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")

        # ---- parameters -------------------------------------------------
        self.declare_parameter("ip",   "192.168.66.10")
        self.declare_parameter("port", 5010)
        ip   = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        # -----------------------------------------------------------------

        self.get_logger().info(f"Connecting to ESP32 at {ip}:{port} …")
        self.trans = Transport(ip, port)
        self.trans.set_sample_callback(self._on_sample)
        self.trans.set_taster_callback(self._on_taster)

        # publishers
        qos = 10
        self.pub_leg  = self.create_publisher(MPU6500Sample, "imu/leg",  qos)
        self.pub_foot = self.create_publisher(MPU6500Sample, "imu/foot", qos)

        # service
        self.create_service(XiaoCmd, "cmd", self.handle_cmd)

        self.get_logger().info("Bridge node ready")

    def _on_taster(self, cid: int, fid: int):
        if fid == 0x01:
            """ Implement Taster actions here """
            self.get_logger().info("Taster 1 pressed")
        elif fid == 0x02:
            self.get_logger().info("Taster 2 pressed")
        else:
            self.get_logger().warn(f"Unknown taster fid {fid:02X}")

    # ================== sample callback =================================
    def _on_sample(self, cid: int, payload: bytes):
        if len(payload) != 16:
            self.get_logger().warn(f"Ignoring sample with bad length {len(payload)}")
            return

        ax, ay, az, gx, gy, gz, ts = struct.unpack("<hhhhhhI", payload)

        msg = MPU6500Sample()
        msg.accel = [ax, ay, az]
        msg.gyro  = [gx, gy, gz]
        msg.ts_ms = ts

        if cid == CID_SENSOR_LEG:
            self.pub_leg.publish(msg)
        elif cid == CID_SENSOR_FOOT:
            self.pub_foot.publish(msg)
        else:
            self.get_logger().warn(f"Unknown sensor CID 0x{cid:02X}")

    # ================== command service =================================
    def handle_cmd(self, request, response):
        try:
            reply = self.trans.send_frame(request.cid, request.fid, list(request.args))
            response.status  = 0 if reply == "OK" else 1
            response.message = reply
        except Exception as e:
            response.status  = 1
            response.message = str(e)

        return response

# -----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
