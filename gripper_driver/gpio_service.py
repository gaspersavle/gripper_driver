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

SERVICE_NAME_LEFT="gripper_left_open"
SERVICE_NAME_RIGHT="gripper_right_open"
SERVICE_NAME_BOTH="gripper_both_open"
PARAMETER_NAME = "gripper_delay"

class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')
        DEFAULT_DELAY = 4
        self.delay = None
        self.service_left= self.create_service(SetBool, SERVICE_NAME_LEFT, self.gpio_control_callback_L)
        self.service_right = self.create_service(SetBool, SERVICE_NAME_RIGHT, self.gpio_control_callback_R)
        self.service_both = self.create_service(SetBool, SERVICE_NAME_BOTH, self.gpio_control_callback_BOTH)
        self.declare_parameter(PARAMETER_NAME, DEFAULT_DELAY)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(GPIO_PIN_L_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_L_b, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_b, GPIO.OUT)

    def gpio_control_callback_L(self, request, response):
        self.delay = self.get_parameter(PARAMETER_NAME).get_parameter_value().integer_value
        print(f"{Fore.RED}Delay param: {self.delay}{Fore.RESET}")
        if request.data:
            self.open_gripper("left")
            response.success = True
            response.message = f"L gripper OPENED"
        else:
            self.close_gripper("left")
            response.success = True
            response.message = f"L gripper CLOSED"
        return response

    def gpio_control_callback_R(self, request, response):
        self.delay = self.get_parameter(PARAMETER_NAME).get_parameter_value().integer_value
        if request.data:
            self.open_gripper("right")
            response.success = True
            response.message = f"R gripper OPENED"
        else:
            self.close_gripper("right")
            response.success = True
            response.message = f"R gripper CLOSED"
        return response

    def gpio_control_callback_BOTH(self, request, response):
        self.delay = self.get_parameter(PARAMETER_NAME).get_parameter_value().integer_value
        print(f"{Fore.RED}Delay param: {self.delay}{Fore.RESET}")
        if request.data:
            self.open_gripper("both")
            response.success = True
            response.message = f"BOTH grippers OPENED"
        else:
            self.close_gripper("both")
            response.success = True
            response.message = f"BOTH grippers CLOSED"
        return response

    def open_gripper(self, side:str):
        sides = ["left", "right", "both"]
        assert side in sides
        if side == "left":
            GPIO.output(GPIO_PIN_L_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_b, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_L_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_b, GPIO.LOW)
        elif side == "right":
            GPIO.output(GPIO_PIN_R_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_R_b, GPIO.LOW)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_R_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_b, GPIO.LOW)
        else:
            GPIO.output(GPIO_PIN_L_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_b, GPIO.HIGH)
            GPIO.output(GPIO_PIN_R_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_R_b, GPIO.LOW)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_L_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_b, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_b, GPIO.LOW)

    def close_gripper(self, side:str):
        sides = ["left", "right", "both"]
        assert side in sides
        if side == "left":
            GPIO.output(GPIO_PIN_L_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_L_b, GPIO.LOW)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_L_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_b, GPIO.LOW)
        elif side == "right":
            GPIO.output(GPIO_PIN_R_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_b, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_R_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_b, GPIO.LOW)
        else:
            GPIO.output(GPIO_PIN_L_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_L_b, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_b, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_L_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_b, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_b, GPIO.LOW)

def main(args=None):
    rclpy.init()
    gpio_control_node = GPIOControlNode()
    rclpy.spin(gpio_control_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
