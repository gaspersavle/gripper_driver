#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from colorama import Fore
import RPi.GPIO as GPIO
import time

GPIO_PIN_L_a = 17
GPIO_PIN_L_b = 27
GPIO_PIN_R_a = 2
GPIO_PIN_R_b = 3

SERVICE_NAME = "gripper_open"
PARAMETER_NAME = "gripper_delay"

class GPIOControlNode:
    def __init__(self):
        DEFAULT_DELAY = 4
        rospy.init_node('gpio_control_node')

        self.delay = rospy.set_param(PARAMETER_NAME, DEFAULT_DELAY)
        
        self.service = rospy.Service(SERVICE_NAME, SetBool, self.gpio_control_callback)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_PIN_L_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_L_b, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_b, GPIO.OUT)

        rospy.loginfo('GPIO Control Node has been started.')

    def gpio_control_callback(self, request):
        self.delay = rospy.get_param(PARAMETER_NAME, self.delay)
        rospy.loginfo(f"{Fore.RED}Delay param: {self.delay}{Fore.RESET}")

        if request.data:
            self.open_grippers()
            return SetBoolResponse(success=True, message="Gripper OPENED")
        else:
            self.close_grippers()
            return SetBoolResponse(success=True, message="Gripper CLOSED")

    def open_grippers(self):
        rospy.loginfo('Opening gripper...')
        GPIO.output(GPIO_PIN_L_a, GPIO.HIGH)
        GPIO.output(GPIO_PIN_L_b, GPIO.HIGH)
        time.sleep(self.delay)
        GPIO.output(GPIO_PIN_L_a, GPIO.LOW)
        GPIO.output(GPIO_PIN_L_b, GPIO.LOW)

    def close_grippers(self):
        rospy.loginfo('Closing gripper...')
        GPIO.output(GPIO_PIN_R_a, GPIO.HIGH)
        GPIO.output(GPIO_PIN_R_b, GPIO.HIGH)
        time.sleep(self.delay)
        GPIO.output(GPIO_PIN_R_a, GPIO.LOW)
        GPIO.output(GPIO_PIN_R_b, GPIO.LOW)

    def cleanup(self):
        rospy.loginfo('Cleaning up GPIO...')
        GPIO.cleanup()

if __name__ == '__main__':
    node = GPIOControlNode()
    rospy.on_shutdown(node.cleanup)
    rospy.spin()
