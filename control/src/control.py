#!/usr/bin/env python3
"""
MCAV - MiniDrone

Transforms received twist messages into motor pwm signals
"""

import rospy
from PWMController import PWMController

if __name__ == "__main__":

    # initialise ros node
    rospy.init_node("mini_pwm_controller", log_level=rospy.INFO)

    # run pwm controller
    pwm_controller = PWMController()
    pwm_controller.drive()
