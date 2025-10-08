#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from threading import Event
import time

from tcp_msg.msg import MPU6500Sample
from tcp_msg.srv import XiaoCmd


def run_test():
    rclpy.init()
    node = Node("bridge_test_client")

    # ============ Sample Test ============
    leg_event = Event()
    foot_event = Event()

    def on_leg(msg):
        scale_accel = 1.0 / 16384.0 # Scale factor for MPU-6500 accelerometer
        scale_gyro  = 1.0 / 131.0  # if FS_SEL = 0
        accel_g = [a * scale_accel for a in msg.accel]
        gyro_dps = [g * scale_gyro for g in msg.gyro]
        node.get_logger().info(f"[LEG] Accel[g]: {accel_g}, Gyro[dps]: {gyro_dps}, ts: {msg.ts_ms}")
        leg_event.set()

    def on_foot(msg):
        scale_accel = 1.0 / 16384.0 # Scale factor for MPU-6500 accelerometer
        scale_gyro  = 1.0 / 131.0  # if FS_SEL = 0
        accel_g = [a * scale_accel for a in msg.accel]
        gyro_dps = [g * scale_gyro for g in msg.gyro]
        node.get_logger().info(f"[FOOT] Accel[g]: {accel_g}, Gyro[dps]: {gyro_dps},, ts: {msg.ts_ms}")
        foot_event.set()

    node.create_subscription(MPU6500Sample, "imu/leg", on_leg, 10)
    node.create_subscription(MPU6500Sample, "imu/foot", on_foot, 10)

    node.get_logger().info("Waiting for IMU samples...")
    timeout = time.time() + 5
    while not (leg_event.is_set() and foot_event.is_set()) and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not leg_event.is_set() or not foot_event.is_set():
        node.get_logger().error("IMU samples not received in time.")
        rclpy.shutdown()
        return
    node.get_logger().info("IMU data received.")

    # ============ Ankle Commands ============
    client = node.create_client(XiaoCmd, "cmd")

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn("Waiting for service /cmd...")

    def send_cmd(cid, fid):
        req = XiaoCmd.Request()
        req.cid = cid
        req.fid = fid
        req.args = []
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        return future.result()
    
    node.get_logger().info("Locking ankle...")
    result = send_cmd(0x03, 0x01)
    node.get_logger().info(f"Result: {result.message}")
    time.sleep(4)

    node.get_logger().info("Unlocking ankle...")
    result = send_cmd(0x03, 0x02)
    node.get_logger().info(f"Result: {result.message}")
    time.sleep(4)
    
    # ============ Taster Check ============
    taster1 = Event()
    taster2 = Event()

    def taster_callback(cid, fid):
        if fid == 0x01:
            node.get_logger().info("Taster 1 pressed")
            taster1.set()
        elif fid == 0x02:
            node.get_logger().info("Taster 2 pressed")
            taster2.set()

    # Connect to bridge_node's Transport directly is not possible here,
    # so we just prompt user and wait for the bridge to print messages.
    node.get_logger().info("Waiting for taster presses... (Check bridge_node log output)")
    input("Press Taster 1 and press ENTER...")
    input("Press Taster 2 and press ENTER...")

    node.get_logger().info("Test sequence complete.")
    rclpy.shutdown()


if __name__ == "__main__":
    run_test()
