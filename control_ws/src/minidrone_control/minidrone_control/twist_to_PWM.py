#! /usr/bin/env/python3
import board
import busio
import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String

class PWM_Controller(Node):

    def __init__(self):
        super().__init__('pwm_controller')
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

        self.MAX_SPEED = PWM_Controller.mini_max_speed_pwm #self.get_param("mini_max_speed_pwm")
        self.MIN_SPEED = 10

        # initialise control values
        self.speed = 0
        self.steering = (self.MIN_STEERING_PWM +
                         self.MAX_STEERING_PWM) / 2  # midpoint


def main(args=None):
    rclpy.init(args=args)

    pwm_controller = PWM_Controller()

    rclpy.spin(pwm_controller)

    pwm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
