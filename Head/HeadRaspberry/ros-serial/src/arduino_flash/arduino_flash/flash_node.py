import rclpy
from rclpy.node import Node

from flash_msg.srv import FlashCommand as FlashService
from arduino_flash.Flash import Flash as FlashHelper
from arduino_flash.locks import get_port_lock

class FlashNode(Node):
    def __init__(self):
        super().__init__('flash_node')
        self.flash_helper = FlashHelper()
        self.srv = self.create_service(
            FlashService,
            'flash',
            self.handle_flash
        )
        self.get_logger().info("Flash node ready to receive requests")

    def handle_flash(self, request, response):
        name = request.name

        # figure out which port this device uses
        port_attr = f'port{name}'
        if not hasattr(self.flash_helper, port_attr):
            response.success = False
            response.message = f"Unknown device '{name}'"
            return response

        port = getattr(self.flash_helper, port_attr)
        lock = get_port_lock(port)

        # try to reserve the port
        if not lock.acquire(blocking=False):
            response.success = False
            response.message = f"Port busy: {port}"
            return response

        try:
            success, msg = self.flash_helper.flash(name)
            response.success = success
            response.message = msg
        except Exception as e:
            response.success = False
            response.message = f"Exception: {e}"
        finally:
            lock.release()

        return response

def main(args=None):
    rclpy.init(args=args)
    node = FlashNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()