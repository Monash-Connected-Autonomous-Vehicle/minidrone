#! /usr/bin/env/python3
import board
import busio
import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from geometry_msgs.msg import Twist, TwistStamped

# TODO: add a way to change the max speed while running
# - press a button on controller to unlock higher speed?

class PWMController():

    """PWMController
    Class to transform recieved twist messages (cmd_vel) to motor pwm signals.
    """
    mini_max_speed_pwm = 40 # hardcoded parameters for initial conversion test
    def __init__(self):
        # initialise i2c and servokit
        self.get_logger().info("Initializing i2c")
        self.i2c_bus0 = busio.I2C(board.SCL, board.SDA)

        self.get_logger().info("Initializing ServoKit")
        self.kit = ServoKit(channels=16, i2c=self.i2c_bus0)
        # kit[0] is ESC for motor control
        # kit[1] is the steering servo
        self.get_logger().info("Done initializing")

        # initialise ros node and sub
        self.create_subscription(Twist, "/twist_mux/cmd_vel", self.twist_callback)
        # self.set_param("mini_max_speed_pwm", 40)
        
        # set controller parameters
        self.MIN_STEERING_PWM = 40  # left steering is a bit broken
        self.MAX_STEERING_PWM = 90

        self.MAX_SPEED = PWMController.mini_max_speed_pwm #self.get_param("mini_max_speed_pwm")
        self.MIN_SPEED = 10

        # initialise control values
        self.speed = 0
        self.steering = (self.MIN_STEERING_PWM +
                         self.MAX_STEERING_PWM) / 2  # midpoint

    def scale(self, val, src, dst):
        """
        Scale the given value from the scale of src to the scale of dst.

        Args:
            val = the current value to be scaled.
            src = the min and max values of the source value.
            dst = the min and max values of the target value.

        Returns:
            scaled_val =  the rescaled value in the scale of dst.
        """
        return ((val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

    def twist_callback(self, msg):
        """Transform the twist velocities to motor pwm signals"""

        # get max speed from param
        self.MAX_SPEED = PWMController.mini_max_speed_pwm#rospy.get_param("mini_max_speed_pwm")

        # speed (no brakes)
        if msg.linear.x >= 0:
            self.speed = self.scale(
                msg.linear.x, [0.0, 1.0], [self.MIN_SPEED, self.MAX_SPEED]
            )
            self.kit.servo[0].angle = self.speed

        # steering
        self.steering = self.scale(
            msg.angular.z,
            [-1.0, 1.0],
            [self.MIN_STEERING_PWM, self.MAX_STEERING_PWM],
        )
        self.kit.servo[1].angle = self.steering

    def drive(self):
        """
        Start the motor controller and ROS loop.
        Graceful shutdown of the vehicle on exit
        """

        try:
            # set speed to 0 to arm ESC
            self.kit.servo[0].angle = 0
            rclpy.spin(self) # TBC?

        finally:
            # Set speed to 0 and steering to straight ahead")
            self.kit.servo[0].angle = 0
            self.kit.servo[1].angle = self.scale(
                0, [-1.0, 1.0], [self.MIN_STEERING_PWM, self.MAX_STEERING_PWM])
