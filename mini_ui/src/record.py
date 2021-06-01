#!/usr/bin/env python3


import rosbag
import rospy
from std_msgs.msg import Int32, String
from sensor_msgs.msg import CompressedImage

from datetime import datetime 


# When button is pressed, start recording
# initialise class when ui gets initialised?


class RosbagRecorder():

    """Helper class for recording rosbag data """

    def __init__(self):

        self.DATA_PATH = ""        # todo: need to set this absolutely for roslaunch...
        self.ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        self.bag = rosbag.Bag(self.DATA_PATH + self.ts + ".bag", "w")
        self.bag_name = self.bag.filename
        
        self.idx = 0
        self.recording = False
        self.bag_closed = False

        self.record_sub = rospy.Subscriber("/jetbot_camera/0/compressed", CompressedImage, self.record_callback, queue_size=1)

        print("Bag Filename: ", self.bag.filename)

    def stop_recording(self):
        
        """ Stop the recording and close the bag to stop data corruption"""
        self.recording = False
        self.bag_closed = True
        self.bag.close()


    def record_callback(self, msg):

        # print("Bag Mode: ", self.bag.mode)
        if self.recording:

            # open bag in append mode if closed
            if self.bag_closed == True:
                self.bag = rosbag.Bag(self.bag_name, "a") 
                self.bag_closed = False

            # img
            self.bag.write("/jetbot_camera/0/compressed", msg)
            # Python too slow?        
        
        # TODO: change to a message_filter for synchronised recording


    def read_bag(self):
        
        self.bag = rosbag.Bag(self.bag_name, "r")
        for topic, msg, t in self.bag.read_messages(topics=['chatter', 'numbers']):
            print(msg)
        self.bag.close()


if __name__ == "__main__":


    rospy.init_node("recorder_test")
    
    recorder = RosbagRecorder()
    print("recording test")




# http://wiki.ros.org/rosbag/Code%20API#Python_API
# pip3 install pycryptodomex python-gnupg