import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray
#from iot_humans_track.msg import int_array2d
#from iot_humans_track.msg import int_array1d
from cv_bridge import CvBridge


class Sense(Node):

    def __init__(self):
        super().__init__('sense')
        '''
        create a subscriber /custom_ns/depth_camera/image_raw
        '''
        self.subscription = self.create_subscription(
            Image,       
        self.subscription)  # prevent unused variable warning
        self.publisher_1 = self.create_publisher(Float32MultiArray, '/lane_lines', 10)
        self.publisher_ = self.create_publisher(Image, '/test_md_ld', 10)
        
    
    def make_points(self, image, line):
      #slope, intercept = line[0], line[1]
      slope, intercept = line
      y1 = int(image.shape[0])# bottom of the image
      y2 = int(y1*3/5)         # slightly lower than the middle
      x1 = int((y1 - intercept)/slope)
      x2 = int((y2 - intercept)/slope)
      return [[x1, y1, x2, y2]]

    def average_slope_intercept(self, image, lines):
        left_fit    = []
        right_fit   = []
        if lines is None:
            return None
        for line in lines:
            for x1, y1, x2, y2 in line:
                fit = np.polyfit((x1,x2), (y1,y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0: # y is reversed in image
                    left_fit.append((slope, intercept))         
                else:
                    right_fit.append((slope, intercept))   
        # add more weight to longer lines
        if len(left_fit) and len(right_fit):
            left_fit_average  = np.average(left_fit, axis=0)
            right_fit_average = np.average(right_fit, axis=0)
            left_line  = self.make_points(image, left_fit_average)
            right_line = self.make_points(image, right_fit_average)
            averaged_lines = [left_line, right_line]
            return averaged_lines

    def canny(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        kernel = 5
        blur = cv2.GaussianBlur(gray,(kernel, kernel),0)
        canny = cv2.Canny(gray, 50, 150)
        return canny

    def display_lines(self,img,lines):
        line_image = np.zeros_like(img)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    #x1 = int(line[0][0])
                    #y1 = int(line[0][1])
                    #x2 = int(line[0][2])
                    #y2 = int(line[0][3])           
                    cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),100)
        return line_image

    def region_of_interest(self, canny):
        height = canny.shape[0]
        width = canny.shape[1]
        mask = np.zeros_like(canny)

        triangle = np.array([[
        (0,height),
        (0,height*0.55),
        (width,height*0.55),
        (width,height)]], np.int32)

        cv2.fillPoly(mask, triangle, 255)
        masked_image = cv2.bitwise_and(canny, mask)
        return masked_image
        
    def find_lane_average(self, msg):
        '''
        follow lane lines and publish image
        '''

        print(msg)
        lineL, lineR = msg[0], msg[1]
        gradL = (-lineL[0][3]+lineL[0][1])/(lineL[0][2]-lineL[0][0])
        gradR = (-lineR[0][3]+lineR[0][1])/(lineR[0][2]-lineR[0][0])
        angleL = np.arctan(gradL)*180/np.pi
        angleR = 180-np.arctan(-gradR)*180/np.pi
        avg_angle = (angleL+angleR)/2
        print(angleL)
        print(angleR)
        print(avg_angle)


    def lane_detect_callback(self, msg):
        '''
        detect lane lines and publish image
        '''
        try:
          print("start")
          bridge = CvBridge()
          cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')       
          canny_image = self.canny(cv_image)
          cropped_canny = self.region_of_interest(canny_image)
          lines = cv2.HoughLinesP(cropped_canny, 2, np.pi/180, 100, np.array([]), minLineLength=40,maxLineGap=5)
          averaged_lines = self.average_slope_intercept(cv_image, lines)
          
#          msg_list = int_list_2d() 
 #         msg_list.data.append(int_list_1d(data=averaged_lines[0]))
  #        msg_list.data.append(int_list_1d(data=averaged_lines[1]))
          self.find_lane_average(averaged_lines)

          line_image = self.display_lines(cv_image, averaged_lines)
          combo_image = cv2.addWeighted(cv_image, 0.8, line_image, 1, 1)
        
          image_msg = bridge.cv2_to_imgmsg(combo_image, encoding="passthrough")
          
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


