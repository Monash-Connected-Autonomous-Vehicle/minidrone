import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from minidrone_control.control_tools import Ackermann, RackAndPinion


class TwistPWMNode(Node):
    def __init__(self):
        super().__init__('twist_pwm_node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.declare_parameter('width', 1.0, 'Width of wheelbase measured from centers of two wheels (m)')
        self.declare_parameter('length', 1.0, 'Distance between front and rear axles (m)')
        self.declare_parameter('steering_ratio', 2.0, 'Ratio between input angle and ideal ackermann steer angle')
        self.declare_parameter('wheel_radius', 0.2, 'Radius of the wheels (m)')
        self.declare_parameter('rack_displacement', 0.2, 'Distance between the front axle and the steering rack (m)')
        self.declare_parameter('link_lengths', [0.5, 0.3, 0.2, 0.0], 'Lengths of each of the 4 links in the (half) steering mechanism, ordered from rack outwards (m)')
        self.declare_parameter('pinion_radius', 0.01, 'Radius of the pinion (m)')

        self.add_on_set_parameters_callback(self.param_update_callback)

        w = self.get_parameter('velocity').value

        self.ackermann = Ackermann()

    def param_update_callback(self, params):
        pass

    def twist_callback(self, msg):
        lin, ang = msg.linear.x, msg.angular.z

        # TODO For each lin, ang: calculate pinion rotation and output



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