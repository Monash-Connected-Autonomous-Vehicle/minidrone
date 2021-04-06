#!/usr/bin/python3

import rospy
import message_filters

from sensor_msgs.msg import Imu, Image, CompressedImage
from sensor_msgs.msg import NavSatFix

""" Script to test the time synchronisation of topics """

def image_callback(data):
    """ Sanity check for images """
    rospy.loginfo(f"image timestamp: {data.header.stamp} ")

def callback(image, imu, fix):

    """ Check: Only returns if all topic timestamps are synched"""

    # check stamps
    rospy.loginfo("------ Topics Synchronised -------")
    rospy.loginfo(f"img timestamp: {image.header.stamp}")
    rospy.loginfo(f"imu timestamp: {imu.header.stamp}")
    rospy.loginfo(f"fix timestamp: {fix.header.stamp}")


def main():

    # ROS setup
    rospy.init_node("test_time_synch_node")
    rospy.loginfo("Node initialised")
 
    # img sanity check
    # rospy.Subscriber('jetbot_camera/0/compressed', CompressedImage, image_callback)
    
    # register subscribers
    image_sub = message_filters.Subscriber('jetbot_camera/0/compressed', CompressedImage)
    imu_sub = message_filters.Subscriber('imu', Imu)
    fix_sub = message_filters.Subscriber('gps', NavSatFix)

    # register time synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, imu_sub, fix_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    
    rospy.spin()

if __name__ == "__main__":

    main()





