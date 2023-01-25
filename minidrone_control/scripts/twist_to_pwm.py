import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from minidrone_control.control_tools import Ackermann, RackAndPinion


class TwistPWMNode(Node):
    def __init__(self):
        super().__init__('twist_pwm_node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

    def twist_callback(self, msg):
        lin, ang = msg.linear.x, msg.angular.z



def main(args=None):
    # Start
    rclpy.init(args=args)
    twist_pwm_node = TwistPWMNode()
    rclpy.spin(twist_pwm_node)
    
    # Stop
    twist_pwm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()