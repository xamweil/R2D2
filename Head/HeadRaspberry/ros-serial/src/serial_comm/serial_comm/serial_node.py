import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from example_interfaces.msg import String as StringMsg

import time 

from serial_msg.srv import DeviceCommand
from serial_comm.Lid import Lid
from serial_comm.Nob import Nob
from serial_comm.Max7219 import Max7219
from serial_comm.LedMatrix import LedMatrix
from serial_comm.SerialProcessor import SerialProcessor
from arduino_flash.locks import get_port_lock
from serial_comm.CameraTiltActuator import CameraTiltActuator
#from serial_comm.SystemTest import SystemTest

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.portUno = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_758343531313513150C1-if00"
        self.portXiao8x8 = "/dev/serial/by-id/usb-Seeed_Seeed_XIAO_M0_3FE8661E5154315432202020FF0B191E-if00"
        self.portXiao4x26 = "/dev/serial/by-id/usb-Seeed_Seeed_XIAO_M0_391D567A5153593336202020FF0A1625-if00"   

        self.serUno     = SerialProcessor(self.portUno)
        self.serXiao8x8  = SerialProcessor(self.portXiao8x8)
        self.serXiao4x26 = SerialProcessor(self.portXiao4x26)

        self.devices = {
                "lid1_1": Lid(self.serUno, 0x01),
                "lid1_2": Lid(self.serUno, 0x02),
                "lid1_3": Lid(self.serUno, 0x03),
                "lid1_4": Lid(self.serUno, 0x04),
                "lid1_5": Lid(self.serUno, 0x05),
                "lid1_6": Lid(self.serUno, 0x06),
                "lid2_1": Lid(self.serUno, 0x07),
                "lid2_2": Lid(self.serUno, 0x08),
                "lid2_3": Lid(self.serUno, 0x09),
                "lid2_4": Lid(self.serUno, 0x0A),
                "lid2_5": Lid(self.serUno, 0x0B),
                "nob1": Nob(self.serUno, 0x10),
                "nob2": Nob(self.serUno, 0x11),
                "nob3": Nob(self.serUno, 0x12),
                "Max7219": Max7219(self.serXiao8x8, 0x01),
                "LedMatrix": LedMatrix(self.serXiao4x26, 0x01),
                "CameraTilt": CameraTiltActuator(self.serUno, 0x20)
            }
            
        self.resp_pub = self.create_publisher(StringMsg, 'serial_response', 10)
        self.srv = self.create_service(DeviceCommand, 'serial_command', self.handle_device_command)

    def handle_device_command(self, request, response):
        """
        request: DeviceCommand.Request with fields:
          - device_name (string)
          - method_name (string)
          - args (int64[])
        response: DeviceCommand.Response with field:
          - response (string)
        """
        dev_name   = request.device_name
        method     = request.method_name
        args_list  = list(request.args)  # list of ints

        # Get correct device:
        dev = self.devices.get(dev_name)

        if not dev:
            response.response = f"Error: Unknown device '{dev_name}'"
            self.get_logger().error(response.response)
            return response
        
        port = dev.ser.port
        lock = get_port_lock(port)
        if lock.locked():
            response.response = f"Error: port {port} is busy (flashing)"
            self.get_logger().warn(response.response)
            return response

        # Check if method exists:
        if not hasattr(dev, method):
            response.response = f"Error: Device '{dev_name}' has no method '{method}'"
            self.get_logger().error(response.response)
            return response

        func = getattr(dev, method)

        try:
            # Call the method with provided args (if any)
            result = func(*args_list) if args_list else func()
            response.response = result
            self.get_logger().info(f"Executed {dev_name}.{method}() → {result}")
        except Exception as e:
            response.response = f"Exception: {e}"
            self.get_logger().error(response.response)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    node.get_logger().info("Serial Node is running...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
        



    