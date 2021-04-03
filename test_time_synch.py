#!/bin/env/python3

import rospy
import message_filters

from sensor_msgs.msg import Imu, Image, CompressedImage
from sensor_msgs.msg import NavSatFix

""" Script to test the time synchronisation of topics """

def image_callback(data):
    """ Sanity check for images """
    rospy.loginfo(f"image timestamp: {data.header.stamp} ")

# NB: jetbot_camera doesnt produce timestmaps :( so not possible to synch like this
# TODO: add timestamps to jetbot_camera
def callback( imu, fix):

    """ only returns if all topic timestamps are synched"""

    # check stamps
    rospy.loginfo("------ Topics Synchronised -------")
    #rospy.loginfo(f"image timestamp {image.header.stamp}")
    rospy.loginfo(f"imu timestamp: {imu.header.stamp}")
    rospy.loginfo(f"fix timestamp: {fix.header.stamp}")


def main():

    rospy.init_node("test_time_synch_node")
    rospy.loginfo("Node initialised")
 
    rospy.Subscriber('jetbot_camera/compressed', CompressedImage, image_callback)
    
    # register subscribers
    #image_sub = message_filters.Subscriber('jetbot_camera/compressed', CompressedImage)
    imu_sub = message_filters.Subscriber('imu', Imu)
    fix_sub = message_filters.Subscriber('gps', NavSatFix)

    # register time synchronizer
    ts = message_filters.ApproximateTimeSynchronizer([imu_sub, fix_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    
    rospy.spin()

if __name__ == "__main__":

    main()





