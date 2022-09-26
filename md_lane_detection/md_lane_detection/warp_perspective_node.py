from calendar import c
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class Sense(Node):

    def __init__(self):
        super().__init__('sense')
        '''
        create a subscriber /custom_ns/depth_camera/image_raw
        '''
        self.subscription = self.create_subscription(
            Image,
            '/stereo_camera/stereo_camera/left/image_raw',
            self.lane_detect_callback,
            10)
        self.publisher_ = self.create_publisher(Image, '/test_md_ld', 10)

    def lane_detect_callback(self, msg):
        '''
        detect lane lines and publish image
        '''
        try:
            print("start")
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')  
            # image = cv_image.astype("float64")
            image = cv2.resize(cv_image, (400,700))
            pts = np.array([[185,540], [235,540], [185, 560], [235,560]], dtype = np.float32)
            # specify input coordinates for corners of red quadrilateral in order TL, TR, BR, BL as x,

            input_pts = np.array([[185,520], [238,520], [185, 580], [235,580]], dtype = np.float32)

            # get top and left dimensions and set to output dimensions of red rectangle

            matrix = cv2.getPerspectiveTransform(np.float32(pts), np.float32(input_pts))
            # do perspective transformation setting area outside input to black
            # Note that output size is the same as the input image size
            imgOutput = cv2.warpPerspective(image, matrix, image.shape[:2][::-1])
            image_msg = bridge.cv2_to_imgmsg(imgOutput, encoding="rgb8")
            self.publisher_.publish(image_msg)
            #self.publisher_1.publish(msg_list)
            print("SUCCESSFUL \n\n")
        except Exception as e:
            print(e)
        

def main(args=None):
    rclpy.init(args=args)
    sense = Sense()
    rclpy.spin(sense)
    sense.destroy_node()
    rclpy.shutdown()

if __name__=="main":
    main()

