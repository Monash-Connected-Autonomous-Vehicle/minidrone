#! /usr/bin/env/python3
import board
import busio
import rospy
from adafruit_servokit import ServoKit
from geometry_msgs.msg import Twist, TwistStamped

# TODO: add a way to change the max speed while running
# - press a button on controller to unlock higher speed?


class PWMController:

    """PWMController
    Class to transform recieved twist messages (cmd_vel) to motor pwm signals.
    """

    def __init__(self):
        # initialise i2c and servokit
        rospy.loginfo("Initializing i2c")
        self.i2c_bus0 = busio.I2C(board.SCL_1, board.SDA_1)
        print(self.i2c_bus0.scan())
        rospy.loginfo("Initializing ServoKit")
        self.kit = ServoKit(channels=16, i2c=self.i2c_bus0)
        # kit[0] is ESC for motor control
        # kit[1] is the steering servo
        rospy.loginfo("Done initializing")

        # initialise ros node and sub
        rospy.Subscriber("/cmd_vel", TwistStamped, self.twist_callback)
        rospy.set_param("mini_max_speed_pwm", 100)

        # set controller parameters
        self.MIN_STEERING_PWM = 55  # left steering is a bit broken
        self.MAX_STEERING_PWM = 135
 
        self.MAX_SPEED = rospy.get_param("mini_max_speed_pwm")
        self.MIN_SPEED = 88

        # initialise control values
        self.speed = 80
        self.steering = (self.MIN_STEERING_PWM + self.MAX_STEERING_PWM) / 2  # midpoint

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
        self.MAX_SPEED = rospy.get_param("mini_max_speed_pwm")

        # speed (no brakes)
        if msg.twist.linear.x >= 0:
            self.speed = self.scale(
                msg.twist.linear.x, [0, 0.5], [self.MIN_SPEED, self.MAX_SPEED]
            )
            print(self.speed)
            self.kit.servo[0].angle = self.speed

        # steering
        self.steering = self.scale(
            msg.twist.angular.z,
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
            self.kit.servo[0].angle = 80
            rospy.spin()

        finally:
            # Set speed to 0 and steering to straight ahead")
            self.kit.servo[0].angle = 80
            self.kit.servo[1].angle = self.scale(0, [-1.0, 1.0], [self.MIN_STEERING_PWM, self.MAX_STEERING_PWM])
