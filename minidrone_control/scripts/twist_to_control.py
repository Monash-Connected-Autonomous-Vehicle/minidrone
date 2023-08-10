#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from minidrone_control.control_tools import Ackermann, RackAndPinion


# TODO: make launch file that runs dynamixel_sdk_examples read_write_node

class TwistToControlNode(Node):
    '''
    A class for converting a twist message (containing linear and angular velocity) into control messages published for both the drving motor, and the pinion (steering) motor
    
    Paramaters
    ----------
    
    width : float
    	Width of wheelbase measured from centers of two wheels (m)
    length : float
    	Distance between front and rear axles (m)
    steering_ratio : float
    	Ratio between input angle and ideal ackermann steer angle. TO BE IMPLEMENTED
    wheel_radius : float
    	Radius of the wheels (m)
    rack_displacement : float
    	Distance between the front axle and the steering rack (m)
    link_length : Tuple[float,float,float,float]
    	Lengths of each of the 4 links in the (half) steering mechanism, ordered from rack outwards (m)
    pinion_radius : float
    	Radius of the pinion (m)
    servo_increment : int
    	Increments per revolution for the steering servo motor
    	
    Subscribers
    -----------
    cmd_vel : L{geometry_msgs.Twist}
    	the linear and angular velocity control signal
    
    Publishers
    ----------
    motor_control : L{std_msgs.Float32}
    	the input for controlling the driving motor (angular velocity of motor shaft)
    	
    set_position : L{dynamixel_sdk_custom_interfaces.set_position}
    	the input message for controling the servo responsible for steering
    
    '''
    def __init__(self):
        super().__init__('twist_to_control_node')

        # ROS2 Parameters
        self.declare_parameter('width', 0.9)  # Width of wheelbase measured from centers of two wheels (m)
        self.declare_parameter('length', 1.2)  # Distance between front and rear axles (m)
        self.declare_parameter('steering_ratio', 2.0)  # Ratio between input angle and ideal ackermann steer angle
        self.declare_parameter('wheel_radius', 0.2)  # Radius of the wheels (m)
        self.declare_parameter('rack_displacement', 0.2)  # Distance between the front axle and the steering rack (m)
        self.declare_parameter('link_lengths', [0.4, 0.15, 0.12, 0.1])  # Lengths of each of the 4 links in the (half) steering mechanism, ordered from rack outwards (m)
        self.declare_parameter('pinion_radius', 0.01)  # Radius of the pinion (m)
        self.declare_parameter('servo_increment', 4096) # Increments per revolution

        self.add_on_set_parameters_callback(self._build_control_tools)

        # ROS2 publishers/subscribers
        self.motor_pub = self.create_publisher(Float32, 'motor_control', 30)
        self.pinion_pub = self.create_publisher(SetPosition, 'set_position', 30)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 30)

        # Non ROS2 functionality
        self._build_control_tools()
                                   
    def _build_control_tools(self):
        try:
            self.ackermann = Ackermann(self.get_parameter('width').value,
                                    self.get_parameter('length').value,
                                    self.get_parameter('wheel_radius').value,
                                    self.get_parameter('steering_ratio').value)
            
            self.rack_and_pinion = RackAndPinion(self.get_parameter('width').value,
                                                self.get_parameter('length').value,
                                                self.get_parameter('wheel_radius').value,
                                                self.get_parameter('link_lengths').value,
                                                self.get_parameter('rack_displacement').value,
                                                self.get_parameter('pinion_radius').value)
        except ValueError:
            self.get_logger().error('Invalid steering mechanism geometry specification')


    def twist_callback(self, msg):
    '''
    FUnction responsible for handling new Twist messages published to cmd_vel topic, and deriving steering and driving commands according to such messages
    '''
    
        lin, ang = msg.linear.x, msg.angular.z
        motor_msg, pinion_msg = Float32(), SetPosition()
        pinion_msg.id = 1
        pinion_msg.position = self.get_parameter('servo_increment').value//2  # Middle position if half way through servo rotation

        if ang == 0: motor_msg.data = self.ackermann.wheel_r*lin
        elif lin == 0: motor_msg.data = 0.0
        else:
            motor_msg.data, th1, th2 = self.ackermann.lin_ang_to_steer_spin(lin, ang)
            servo_inc = self.get_parameter('servo_increment').value/math.pi
            try:
                pinion_msg.position += int(servo_inc*self.rack_and_pinion.steer_to_pinion_ang(th1 if ang > 0 else th2))
            except ValueError:
                self.get_logger().warn('Steering geometry math error!')

        self.get_logger().debug('out %d' % (pinion_msg.position))
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
