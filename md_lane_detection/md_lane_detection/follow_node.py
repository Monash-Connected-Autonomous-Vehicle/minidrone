import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


class Follow(Node):

    def __init__(self):
        super().__init__('follow')
        '''
        '''
        self.get_logger().info('test')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/lane_lines',
            self.lane_follow_callback,
            10)
        self.subscription  # prevent unused variable warning
   
    def lane_follow_callback(self, msg):
      '''
      follow lane lines and publish image
      '''
      print("follow")
      lineL, lineR = msg[0], msg[1]
      gradL = (lineL[3]-lineL[1])/(lineL[2]-lineL[0])
      gradR = (lineR[3]-lineR[1])/(lineR[2]-lineR[0])
      angleL = np.arctan(gradL)*180/np.pi
      angleR = 180-np.arctan(gradR)*180/np.pi
      avg_angle = (angleL+angLeR)/2
      print(angleL)
      print(angleR)
      print(avg_angle)

def main(args=None):
    rclpy.init(args=args)
    follow = Follow()
    rclpy.spin(follow)

    follow.destroy_node()
    rclpy.shutdown()

if __name__=="main":
    main()
      
   
