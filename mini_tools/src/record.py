#!/usr/bin/env python3

from datetime import datetime

import message_filters
import rosbag
import rospy
import yaml
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix
from std_msgs.msg import Bool

# When button is pressed, start recording
# initialise class when ui gets initialised?

class RosbagRecorder:

    """Helper class for recording rosbag data"""

    def __init__(self):

        self.config = yaml.safe_load(
            open(
                "/home/patrick/mcav/catkin_ws/src/minidrone/mini_tools/src/config.yaml"
            )
        )

        self.DATA_PATH = self.config["data"][
            "path"
        ]  # todo: need to set this absolutely for roslaunch...
        self.ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        self.bag = None  # dont setup unless required

        self.idx = 0
        self.recording = False
        self.bag_closed = True

        self.recording_state_sub = rospy.Subscriber(
            "/recorder/recording", Bool, self.recording_state_callback, queue_size=1
        )

        # setup different recording subscribers is using synched mode
        self.cam_topic = self.config["record"]["topics"]["camera"]
        self.imu_topic = self.config["record"]["topics"]["imu"]
        self.gps_topic = self.config["record"]["topics"]["gps"]
        self.twist_topic = self.config["record"]["topics"]["twist"]

        self.SYNCHED_RECORD_MODE = self.config["record"]["use_sync"]
        if self.SYNCHED_RECORD_MODE:

            # register subscribers
            image_sub = message_filters.Subscriber(self.cam_topic, CompressedImage)
            imu_sub = message_filters.Subscriber(self.imu_topic, Imu)
            fix_sub = message_filters.Subscriber(self.gps_topic, NavSatFix)

            # register time synchronizer
            ts = message_filters.ApproximateTimeSynchronizer(
                [image_sub, imu_sub, fix_sub],
                10,
                0.1,
                allow_headerless=True,  # TODO: maybe headerless means we can sync Twist?
            )
            ts.registerCallback(self.synched_callback)

        else:

            self.record_cam_sub = rospy.Subscriber(
                self.cam_topic,
                CompressedImage,
                self.record_cam_callback,
                queue_size=1,
            )
            self.record_imu_sub = rospy.Subscriber(
                self.imu_topic,
                Imu,
                self.record_imu_callback,
                queue_size=1,
            )
            self.record_gps_sub = rospy.Subscriber(
                self.gps_topic,
                NavSatFix,
                self.record_gps_callback,
                queue_size=1,
            )
            self.record_twist_sub = rospy.Subscriber(
                self.twist_topic,
                Twist,
                self.record_twist_callback,
                queue_size=1,
            )

    def create_initial_bag(self):
        self.bag = rosbag.Bag(self.DATA_PATH + self.ts + ".bag", "w")
        self.bag_name = self.bag.filename
        self.bag_closed = False

        rospy.loginfo("Bag Filename: %s", self.bag.filename)

    def setup_bag_for_recording(self):

        if self.bag is None:
            self.create_initial_bag()

        # open bag in append mode if closed
        if self.bag_closed == True:
            self.bag = rosbag.Bag(self.bag_name, "a")
            self.bag_closed = False

    def stop_recording(self):

        """Stop the recording and close the bag to stop data corruption"""
        if self.bag_closed != True:
            self.recording = False
            self.bag_closed = True
            self.bag.close()

    def record_cam_callback(self, msg):
        """Write the contents of the camera msg to the bag"""
        if self.recording:

            # check bag is setup for recording
            self.setup_bag_for_recording()

            # write data
            self.bag.write(self.cam_topic, msg)
            # Python too slow?

    def record_imu_callback(self, msg):
        """Write the contents of the imu msg to the bag"""

        if self.recording:

            # check bag is setup for recording
            self.setup_bag_for_recording()

            # write data
            self.bag.write(self.imu_topic, msg)

    def record_gps_callback(self, msg):
        """Write the contents of the gps msg to the bag"""

        if self.recording:

            # check bag is setup for recording
            self.setup_bag_for_recording()

            # write data
            self.bag.write(self.gps_topic, msg)

    def record_twist_callback(self, msg):
        """Write the contents of the twist msg to the bag"""

        if self.recording:

            # check bag is setup for recording
            self.setup_bag_for_recording()

            # write data
            self.bag.write(self.twist_topic, msg)

    def recording_state_callback(self, msg):

        # if recording is true, start recording
        if msg.data:
            self.recording = True

        # otherwise, do the stop recording proces
        else:
            self.stop_recording()

    def read_bag(self):

        self.bag = rosbag.Bag(self.bag_name, "r")
        for topic, msg, t in self.bag.read_messages():
            print(topic, msg)
        self.bag.close()

    def synched_callback(self, img, imu, fix):

        # record synchronised topics
        # TODO: problem with synched topics: imu and gps too slow
        # gps: 5hz, imu: 8hz, cam: 30hz
        # makes the camera recording only 5 hz (max)
        # in practice much lower (1hz), because only some of them are synchronised
        # TODO: cant use Twist message here either

        if self.recording:

            # check bag is setup for recording
            self.setup_bag_for_recording()

            # img
            self.bag.write(self.cam_topic, img)
            self.bag.write(self.imu_topic, imu)
            self.bag.write(self.gps_topic, fix)
            # Python too slow?


if __name__ == "__main__":

    rospy.init_node("rosbag_recorder")
    rospy.loginfo("Rosbag Recorder Started")

    recorder = RosbagRecorder()

    rospy.spin()


# TODO: change imitation learner to use this recorder class


# http://wiki.ros.org/rosbag/Code%20API#Python_API
# pip3 install pycryptodomex python-gnupg
