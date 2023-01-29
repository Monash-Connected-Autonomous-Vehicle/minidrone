import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from minidrone_control.control_tools import Ackermann, RackAndPinion


class TwistToControlNode(Node):
    '''
    TODO
    '''
    def __init__(self):
        super().__init__('twist_to_control_node')

        # ROS2 Parameters
        self.declare_parameter('width', 1.0, 'Width of wheelbase measured from centers of two wheels (m)')
        self.declare_parameter('length', 1.0, 'Distance between front and rear axles (m)')
        self.declare_parameter('steering_ratio', 2.0, 'Ratio between input angle and ideal ackermann steer angle')
        self.declare_parameter('wheel_radius', 0.2, 'Radius of the wheels (m)')
        self.declare_parameter('rack_displacement', 0.2, 'Distance between the front axle and the steering rack (m)')
        self.declare_parameter('link_lengths', [0.5, 0.3, 0.2, 0.0], 'Lengths of each of the 4 links in the (half) steering mechanism, ordered from rack outwards (m)')
        self.declare_parameter('pinion_radius', 0.01, 'Radius of the pinion (m)')

        self.add_on_set_parameters_callback(self._build_control_tools)

        # ROS2 publishers/subscribers
        self.motor_pub = self.create_publisher(Float32, 'motor_control')
        self.pinion_pub = self.create_publisher(Float32, 'pinion_control')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 30)

        # Other important objects
        self.ackermann, self.rack_and_pinion = self._build_control_tools()
                                   
    def _build_control_tools(self):
        self.ackermann = Ackermann(self.get_parameter('width').value,
                                   self.get_parameter('length').value,
                                   self.get_parameter('wheel_radius').value,
                                   self.get_parameter('steering_ratio').value)
        
        self.rack_and_pinion = RackAndPinion(self.get_parameter('width').value,
                                             self.get_parameter('length').value,
                                             self.get_parameter('wheel_radius').value,
                                             self.get_parameter('link_lengths').value,
                                             self.get_parameter('pinion_radius').value)

    def twist_callback(self, msg):
        lin, ang = msg.linear.x, msg.angular.z
        motor_msg, pinion_msg = Float32(), Float32()

        if ang == 0: motor_msg.data, pinion_msg.data = self.ackermann.wheel_r*lin, 0.0
        elif lin == 0: motor_msg.data, pinion_msg.data = 0.0, 0.0
        else:
            motor_msg.data, th1, th2 = self.ackermann.lin_ang_to_steer_spin(lin, ang)
            pinion_msg.data = self.rack_and_pinion.steer_to_disp(th1 if ang > 0 else th2)

        self.motor_pub.publish(motor_msg)
        self.pinion_pub.publish(pinion_msg)

        
def main(args=None):
    # Start
    rclpy.init(args=args)
    twist_to_control_node = TwistToControlNode()
    rclpy.spin(twist_to_control_node)
    
    # Stop
    twist_to_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()