#!/usr/bin/env python3

import cv2
import cv_bridge
import pandas as pd
import PIL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image, Joy
import torch
import torch.nn as nn
import torchvision
from torchvision import transforms

import os

class ImitationLearning:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.speed = 0
        self.steer = 0
        self.img = None
        self.img_idx = 0

        # unused yet
        self.SAVE_DIR = "data/"
        self.record = False  # recording toggle
        self.autopilot = True  # autopilot toggle

        self.df = pd.DataFrame(columns=["ts", "fname", "speed", "steer"])

        # pytorch setup
        print("Model Setup Start")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torchvision.models.resnet18(pretrained=True)
        num_fts = self.model.fc.in_features
        self.model.fc = nn.Linear(num_fts, 1)
        self.model = self.model.to(self.device)
        self.model.eval()
        print("Model Setup End")
        print(f"Record: {self.record}, Autopilot: {self.autopilot}")

        
        rospy.Subscriber(
            "jetbot_camera/0/compressed_throttle", CompressedImage, self.image_callback
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
        print(f"Img Received: {self.ts}")
        if self.record:
            self.save_data()

        if self.autopilot:
            self.model_inference()

    def model_inference(self):
        #print("model inference")
        transformation = transforms.Compose([transforms.ToPILImage(), transforms.Resize((224, 224)), transforms.ToTensor()])
        img_t = transformation(self.img).unsqueeze(0)
        img_t = img_t.to(self.device)

        print(f"Img Shape: {img_t.shape}")

        steering = self.model(img_t)

        print(f"Steering Output: {steering.item()}")
        # set steering value

    def twist_callback(self, data):

        # assign twist values
        self.speed = data.linear.x
        self.steer = data.angular.z

    def save_data(self):

        fname = f"/home/jetson02/mcav/catkin_ws/src/minidrone/imitation_learning/data/{self.img_idx:09d}.png"
        print(fname)
        # TODO: This can be improved
        db = {"ts": self.ts, "fname": fname, "speed": self.speed, "steer": self.steer}

        self.df = pd.concat(
            [self.df, pd.DataFrame.from_dict(db, orient="index").T], ignore_index=True
        )

        # print(self.df.tail())

        # save image
        cv2.imwrite(fname, self.img)

        # save entire df (dumb, only need increment)
        self.df.to_csv("/home/jetson02/mcav/catkin_ws/src/minidrone/imitation_learning/data/data.csv")

        self.img_idx += 1




if __name__ == "__main__":

    rospy.init_node("mini_imitation_learning")
    rospy.loginfo("Hello Imitation")

    imitiation = ImitationLearning()

    rospy.spin()


# TODO: i would love TwistStamped to be able to use message_filters
# TODO: functionality to start / stop recording (button press?)

# TODO: TOO SLOW


# RUn
# roslaunch control manual_control.launch 
#roslaunch jetbot_ros jetbot_cam_compressed_one.launch 
# rosrun imitation_learning imitation_learning.py