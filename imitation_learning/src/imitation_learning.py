#!/usr/bin/env python3

import cv2
import cv_bridge
import pandas as pd
import PIL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image, Joy


class ImitationLearning:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.speed = 0
        self.steer = 0
        self.img = None
        self.img_idx = 0

        # unused yet
        self.SAVE_DIR = "data/"
        self.record = True  # recording toggle
        self.autopilot = False  # autopilot toggle

        self.df = pd.DataFrame(columns=["ts", "fname", "speed", "steer"])

        rospy.Subscriber(
            "jetbot_camera/0/compressed", CompressedImage, self.image_callback
        )
        rospy.Subscriber("twist_mux/cmd_vel", Twist, self.twist_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)


    def joy_callback(self, data):

        # if square pressed toggle recording 
        if data.buttons[0]:
            self.record = True if not self.record else False

        # if triangle pressed, toggle autopilot
        if data.buttons[3]:
            self.autopilot = True if not self.autopilot else False


        if data.buttons[2]:
            #show data
            print(f"Record: {self.record}, Autopilot: {self.autopilot}")
            print(f"Recorded Data: {len(self.df)}")


    def image_callback(self, data):

        img = self.bridge.compressed_imgmsg_to_cv2(data)

        # synch images and twist
        self.img = img
        self.ts = data.header.stamp

        if self.record:
            self.save_data()

    def twist_callback(self, data):

        # assign twist values
        self.speed = data.linear.x
        self.steer = data.angular.z

    def save_data(self):

        fname = f"../data/{self.img_idx:09d}.png"

        # TODO: This can be improved
        db = {"ts": self.ts, "fname": fname, "speed": self.speed, "steer": self.steer}

        self.df = pd.concat(
            [self.df, pd.DataFrame.from_dict(db, orient="index").T], ignore_index=True
        )

        print(self.df.tail())

        # save image
        cv2.imwrite(fname, self.img)

        # save entire df (dumb, only need increment)
        self.df.to_csv("../data/data.csv")

        self.img_idx += 1




if __name__ == "__main__":

    rospy.init_node("mini_imitation_learning")
    rospy.loginfo("Hello Imitation")

    imitiation = ImitationLearning()

    rospy.spin()


# TODO: i would love TwistStamped to be able to use message_filters
# TODO: functionality to start / stop recording (button press?)

# TODO: TOO SLOW
