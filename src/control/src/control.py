#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped

from adafruit_servokit import ServoKit
import board
import busio
import time

print("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL, board.SDA))
print("Initializing ServoKit")
kit = ServoKit(channels=16, i2c=i2c_bus0)
# kit[0] is ESC for motor control
# kit[1] is the steering servo
print("Done initializing")

MIN_STEERING_PWM = 40
MAX_STEERING_PWM = 90

MAX_SPEED = 90
MIN_SPEED = 10

speed = 0
steering = (MIN_STEERING_PWM + MAX_STEERING_PWM) / 2

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

def twist_callback(message):
    throttle = message.linear.x
    if throttle >= 0:
        speed = scale(throttle, [0.0, 1.0], [MIN_SPEED, MAX_SPEED])
        kit.servo[0].angle = speed

    steering = scale(message.angular.z, [-1.0, 1.0], [MIN_STEERING_PWM, MAX_STEERING_PWM])
    kit.servo[1].angle = steering

def drive():
    rospy.init_node("drive")
    rospy.Subscriber("/cmd_vel", TwistStamped, twist_callback)
    # set speed to 0 to arm ESC
    kit.servo[0].angle = 0
    
    rospy.spin()

if __name__ == '__main__':
    try:
        drive()
    finally:
        # set speed to 0 if program is exited prematurely
        kit.servo[0].angle = 0
