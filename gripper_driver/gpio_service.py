from std_srvs.srv import SetBool
import rclpy
from colorama import Fore
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

GPIO_PIN_L_a = 17
GPIO_PIN_L_b = 27
GPIO_PIN_R_a = 2
GPIO_PIN_R_b = 3

SERVICE_NAME ="gripper_open"
PARAMETER_NAME = "gripper_delay"

class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')
        DEFAULT_DELAY = 4
        self.delay = None
        self.service= self.create_service(SetBool, SERVICE_NAME, self.gpio_control_callback)
        self.declare_parameter(PARAMETER_NAME, DEFAULT_DELAY)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(GPIO_PIN_L_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_L_b, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_b, GPIO.OUT)

    def gpio_control_callback(self, request, response):
        self.delay = self.get_parameter(PARAMETER_NAME).get_parameter_value().integer_value
        print(f"{Fore.RED}Delay param: {self.delay}{Fore.RESET}")
        if request.data:
            self.open_gripper()
            response.success = True
            response.message = f"gripper OPENED"
        else:
            self.close_gripper()
            response.success = True
            response.message = f"gripper CLOSED"
        return response

    def open_gripper(self):
            GPIO.output(GPIO_PIN_L_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_L_b, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_L_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_b, GPIO.LOW)

    def close_gripper(self):
            GPIO.output(GPIO_PIN_R_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_R_b, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_R_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_b, GPIO.LOW)
  
def main(args=None):
    rclpy.init()
    gpio_control_node = GPIOControlNode()
    rclpy.spin(gpio_control_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
