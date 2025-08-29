"""
To test the Xiao Bridge functionality, start the nodes:
# Left
ros2 run xiao_bridge bridge_node --ros-args -p ip:=192.168.66.10 -p port:=5010 -r __ns:=/leg_l &
# Right
ros2 run xiao_bridge bridge_node --ros-args -p ip:=192.168.66.11 -p port:=5011 -r __ns:=/leg_r &


Than run this test scipt for 'left', right' or 'both' legs 
"""

#!/usr/bin/env python3
import argparse
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tcp_msg.msg import MPU6500Sample
from rcl_interfaces.msg import Log as RosLog

# Scale factors for MPU-6500 (assuming FS_SEL=0 for gyro, ±2g for accel)
ACCEL_SCALE = 1.0 / 16384.0  # g per LSB
GYRO_SCALE  = 1.0 / 131.0    # dps per LSB

# Consider a taster "ACTIVE" if we saw a press log within this many seconds
TASTER_ACTIVE_WINDOW_S = 1.5

@dataclass
class TopicStats:
    count: int = 0            # samples seen this second
    last_accel: Tuple[float, float, float] = (float("nan"),) * 3
    last_gyro:  Tuple[float, float, float] = (float("nan"),) * 3
    last_ts_ms: int = 0

@dataclass
class SideState:
    leg: TopicStats = field(default_factory=TopicStats)
    foot: TopicStats = field(default_factory=TopicStats)
    last_t1_press_monotonic: float = -1.0
    last_t2_press_monotonic: float = -1.0

class XiaoDashboard(Node):
    def __init__(self, sides):
        super().__init__("xiao_dashboard_client")
        self.sides = sides  # list like ['leg_l', 'leg_r']
        self.state: Dict[str, SideState] = {s: SideState() for s in self.sides}

        qos = QoSProfile(depth=50)

        # Subscriptions for each side: imu/leg and imu/foot
        for side in self.sides:
            # topics are absolute (include leading slash)
            self.create_subscription(
                MPU6500Sample, f"/{side}/imu/leg",
                lambda msg, s=side: self._on_sample(msg, s, which="leg"),
                qos
            )
            self.create_subscription(
                MPU6500Sample, f"/{side}/imu/foot",
                lambda msg, s=side: self._on_sample(msg, s, which="foot"),
                qos
            )

        # Listen to /rosout to catch “Taster 1 pressed” / “Taster 2 pressed”
        self.create_subscription(RosLog, "/rosout", self._on_log, qos)

        # Redraw timer (1 Hz)
        self.last_draw = time.monotonic()
        self.create_timer(1.0, self._draw_and_reset)

        # Clear screen once
        self._clear_screen()

    # ---------- callbacks ----------
    def _on_sample(self, msg: MPU6500Sample, side: str, which: str):
        stats = self.state[side].leg if which == "leg" else self.state[side].foot
        stats.count += 1
        # scale values
        ax, ay, az = [a * ACCEL_SCALE for a in msg.accel]
        gx, gy, gz = [g * GYRO_SCALE for g in msg.gyro]
        stats.last_accel = (ax, ay, az)
        stats.last_gyro  = (gx, gy, gz)
        stats.last_ts_ms = msg.ts_ms

    def _on_log(self, log: RosLog):
        # We expect messages like "Taster 1 pressed" from nodes:
        # /leg_l/bridge_node or /leg_r/bridge_node
        name = log.name  # e.g., '/leg_l/bridge_node'
        text = log.msg or ""
        now  = time.monotonic()

        for side in self.sides:
            side_node = f"/{side}/bridge_node"
            if name == side_node:
                if "Taster 1 pressed" in text:
                    self.state[side].last_t1_press_monotonic = now
                elif "Taster 2 pressed" in text:
                    self.state[side].last_t2_press_monotonic = now

    # ---------- UI ----------
    def _clear_screen(self):
        sys.stdout.write("\x1b[2J\x1b[H")
        sys.stdout.flush()

    def _draw_and_reset(self):
        now = time.monotonic()
        # Compose a stationary dashboard
        lines = []
        lines.append("Xiao ESP32C3 – IMU & Taster Dashboard (1 Hz)")
        lines.append(f"Updated: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append("-" * 72)

        for side in self.sides:
            st = self.state[side]
            # Taster activity
            t1_active = (now - st.last_t1_press_monotonic) <= TASTER_ACTIVE_WINDOW_S if st.last_t1_press_monotonic >= 0 else False
            t2_active = (now - st.last_t2_press_monotonic) <= TASTER_ACTIVE_WINDOW_S if st.last_t2_press_monotonic >= 0 else False

            # Pretty helpers
            def fmt_vec(v):
                if any(math.isnan(x) for x in v):
                    return "(no data)"
                return f"({v[0]: .3f}, {v[1]: .3f}, {v[2]: .3f})"

            lines.append(f"[{side}]")
            lines.append(f"  Leg : {st.leg.count:3d} Hz | accel[g]={fmt_vec(st.leg.last_accel)}  gyro[dps]={fmt_vec(st.leg.last_gyro)}  ts={st.leg.last_ts_ms}")
            lines.append(f"  Foot: {st.foot.count:3d} Hz | accel[g]={fmt_vec(st.foot.last_accel)}  gyro[dps]={fmt_vec(st.foot.last_gyro)}  ts={st.foot.last_ts_ms}")
            lines.append(f"  Taster1: {'ACTIVE' if t1_active else 'inactive'}   Taster2: {'ACTIVE' if t2_active else 'inactive'}")
            lines.append("")

            # Reset per-second counters
            st.leg.count = 0
            st.foot.count = 0

        # Draw
        self._clear_screen()
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()
        self.last_draw = now


def parse_args():
    p = argparse.ArgumentParser(
        description="Terminal dashboard for Xiao bridge nodes (left/right/both)."
    )
    p.add_argument(
        "--side",
        choices=["left", "right", "both"],
        default="both",
        help="Which leg(s) to monitor (default: both)."
    )
    # Allow overriding namespaces, if ever needed
    p.add_argument("--left-ns", default="leg_l", help="Left leg namespace (default: leg_l)")
    p.add_argument("--right-ns", default="leg_r", help="Right leg namespace (default: leg_r)")
    return p.parse_args()


def main():
    args = parse_args()
    sides = []
    if args.side in ("left", "both"):
        sides.append(args.left_ns)
    if args.side in ("right", "both"):
        sides.append(args.right_ns)

    rclpy.init()
    node = XiaoDashboard(sides)

    # Graceful Ctrl-C
    def handle_sigint(signum, frame):
        rclpy.shutdown()
        # ensure the screen cursor lands on a new line
        print()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
