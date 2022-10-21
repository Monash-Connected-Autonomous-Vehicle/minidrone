import numpy as np
import glob 
import rclpy
import time
import cv2 as cv
import PIL
import math
import os
from rclpy.node import Node
from sensor_msgs.msg import Img
from cv_bridge import CvBridge
from tf2_ros.transform_listener import TransformListener


class Tranformation_ROS_subscriber(Node):
    def __init__(self):
        super().__init__("Tranformation ROS Listener")
        self.leftImg_pub = self.create_subscription(Img, '/img_left', 10)
        self.rightImg_pub = self.create_subscription(Img, '/img_right', 10)
        timer_period = 0.5
        self.spinner = self.create_timer(timer_period, self.spin)

    def get_object(array):
        global img_geo, robot_odometry
        u = array.data[0] + (array.data[2] // 2)
        v = array.data[1] + (array.data[3])

        listener = TransformListener()


        camera_point = img_geo.projectPixelTo3dRay((img_geo.rectifyPoint((u, v))))

        point_msg.pose.position.x = camera_point[0] + robot_odometry.pose.pose.position.x
        point_msg.pose.position.y = camera_point[1] + robot_odometry.pose.pose.position.y
        point_msg.pose.position.z = 0
        point_msg.pose.orientation.x = 0
        point_msg.pose.orientation.y = 0
        point_msg.pose.orientation.z = 0
        point_msg.pose.orientation.w = 1
        point_msg.header.frame_id = img_geo.tfFrame()
        point_msg.header.stamp = time
    
        try:
            listener.waitForTransform(img_geo.tfFrame(), 'map', time, rospy.Duration(1))
            tf_point = listener.transformPose('map', point_msg)
            print(convert_from_robot_to_map(tf_point.pose.position.y, tf_point.pose.position.x))
        except Exception:
            pass

    def convert_from_robot_to_map(robot_y, robot_x):
        global map_info
        map_x = (robot_x - map_info.info.origin.position.x) / map_info.info.resolution
        map_y = (robot_y - map_info.info.origin.position.y) / map_info.info.resolution
        
        return map_y, map_x


def main(args=None):
    rclpy.init(args=args)

    tansformation_ros_subscriber = Tranformation_ROS_subscriber()

    try:
        rclpy.spin(tansformation_ros_subscriber)
    except KeyboardInterrupt:
        tansformation_ros_subscriber.get_logger().debug("Keyboard interrupt")

    # destroy node explicity
    tansformation_ros_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
    
