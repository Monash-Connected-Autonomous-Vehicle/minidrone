#!/usr/bin/env python3
"""
MCAV - MiniDrone

Transforms received twist messages into motor pwm signals
"""

import rclpy
import sys
from PWMController import PWMController

if __name__ == "__main__":

    # initialise ros node
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('mini_pwm_controller') #log_level=rospy.INFO

    # run pwm controller
    pwm_controller = PWMController()
    pwm_controller.drive()
    #rclpy.spin(pwm_controller) #TBC
