import rclpy
from rclpy.node import Node
from bms_reader.BMS import BMS
from sensor_msgs.msg import BatteryState
from dalybms import DalyBMS
import serial

class BMSNode(Node):
    def __init__(self):
        super().__init__("bms_node")

        # ---- parameters -------------------------------------------------
        self.declare_parameter("device", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")  
        self.declare_parameter("publish_rate", 2) # Hz
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        device = self.get_parameter("device").get_parameter_value().string_value

        # -----------------------------------------------------------------

        self.get_logger().info(f"Connecting to BMS at {port} …")
        self.bms = BMS(device=device)

        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self.update)
        self.pub_battery_state = self.create_publisher(BatteryState, "battery_state", 10)

    def update(self):
        stamp = self.get_clock().now().to_msg()
        self.bms.update_all(stamp)
        self.pub_battery_state.publish(self.bms.current_state)

def main(args=None):
    rclpy.init(args=args)
    node = BMSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()