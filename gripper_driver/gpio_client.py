import rclpy
import sys
from std_srvs.srv import SetBool
from rclpy.node import Node

class GPIOControlClient:
    def __init__(self):
        super().__init__('gpio_control_client')
        self.client = self.create_client(SetBool, 'gpio_control')

    def toggle_gpio_pin(self, pin_state):
        request = SetBool.Request()
        request.data = pin_state

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.node.get_logger().info('GPIO pin toggled successfully')
        else:
            self.node.get_logger().error('Failed to toggle GPIO pin')

def main():
    rclpy.init()
    gpio_client = GPIOControlClient()
    pin_state = True  # Set to True to turn on the pin, False to turn off
    gpio_client.toggle_gpio_pin(pin_state)
    gpio_client.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()









    request.data = pin_state

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('GPIO pin toggled successfully')
    else:
        node.get_logger().error('Failed to toggle GPIO pin')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    pin_state = True  # Set to True to turn on the pin, False to turn off
    toggle_gpio_pin(pin_state)
