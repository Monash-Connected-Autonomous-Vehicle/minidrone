from calendar import c
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class Camera_Cut(Node):

    def __init__(self):
        super().__init__('camera_cut')
        '''
        create a subscriber /camera_info
        '''
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_cut_callback,
            10)
        self.publisher_ = self.create_publisher(Image, '/test_md_ld', 10)

    def camera_cut_callback(self, msg):
        '''
        cut the camera image and publish image
        '''
        try:
            print("start")
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') 
            cv_image = cv_image[0:376, 0:672]
            cv2.imshow('image',cv_image)
            # image = cv_image.astype("float64")
            image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
            self.publisher_.publish(image_msg)

            #self.publisher_1.publish(msg_list)
            print("SUCCESSFUL \n\n")
        except Exception as e:
            print(e)
        

def main(args=None):
    rclpy.init(args=args)
    sense = Camera_Cut()
    rclpy.spin(sense)
    sense.destroy_node()
    rclpy.shutdown()

if __name__=="main":
    main()

