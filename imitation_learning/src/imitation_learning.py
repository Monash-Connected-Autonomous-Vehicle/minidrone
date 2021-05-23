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

import PIL


class ImitationLearner:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.speed = 0
        self.steer = 0
        self.img = None
        self.img_idx = 0

        # config
        self.SAVE_DIR = (
            "/home/jetson03/mcav/catkin_ws/src/minidrone/imitation_learning/data/"
        )
        self.MODEL_PATH = (
            "/home/jetson03/mcav/catkin_ws/src/minidrone/imitation_learning/models/models.pt"
        )
        self.record = False  # recording toggle
        self.autopilot = True  # autopilot toggle

        # try to load the database, otherwise create new
        try:
            self.df = pd.read_csv(self.SAVE_DIR + "data.csv")
        except:
            self.df = pd.DataFrame(columns=["ts", "fname", "speed", "steer"])

        # pytorch setup
        print("Model Setup Start")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torchvision.models.resnet18(pretrained=True)
        num_fts = self.model.fc.in_features
        self.model.fc = nn.Linear(num_fts, 1)
        self.model = self.model.to(self.device)

        # load trained model        
        try:
            self.model.load_state_dict(torch.load(self.MODEL_PATH))
        except:
            pass

        self.model.eval()
        print("Model Setup End")
        print(f"Record: {self.record}, Autopilot: {self.autopilot}")

        # subscriber setup
        rospy.Subscriber(
            "jetbot_camera/0/compressed_throttle", CompressedImage, self.image_callback
        )
        rospy.Subscriber("twist_mux/cmd_vel", Twist, self.twist_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # publisher setup
        self.twist_pub = rospy.Publisher("imitation_vel", Twist, queue_size=1)

    def joy_callback(self, data):

        # if square pressed toggle recording
        if data.buttons[0]:
            self.record = True if not self.record else False

        # if triangle pressed, toggle autopilot
        if data.buttons[3]:
            self.autopilot = True if not self.autopilot else False

            # stop recording if autopilot (just for performance sake atm)
            if self.autopilot:
                self.record = False

        # if circle pressed, show data
        if data.buttons[2]:
            print(f"Record: {self.record}, Autopilot: {self.autopilot}")
            print(f"Recorded Data: {len(self.df)}")
            print(self.df.tail())
            print("-"*50)

    def image_callback(self, data):

        img = self.bridge.compressed_imgmsg_to_cv2(data)

        # reverse image colour channels (bgr to rgb)
        img = img[:, :, ::-1]

        # synch images and twist
        self.img = img
        self.ts = data.header.stamp
        print(f"Img Received: {self.ts}")
        if self.record:
            self.save_data()

        if self.autopilot:
            self.model_inference()

    def model_inference(self):
        """ Pass the current image through the model to predict required steering / speed"""
        
        # image pre-processing
        transformation = transforms.Compose(
            [
                transforms.ToPILImage(),
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
            ]
        )
        img_t = transformation(self.img).unsqueeze(0)
        img_t = img_t.to(self.device)

        # model forward pass
        steering = self.model(img_t)

        print(f"Img Shape: {img_t.shape}")
        print(f"Steering Output: {steering.item()}")

        # publish steering value
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        twist_msg.angular.z = steering.item()

        self.twist_pub.publish(twist_msg)

    def twist_callback(self, data):
        """ Set steering and speed values from twist callback"""

        # assign twist values
        self.speed = data.linear.x
        self.steer = data.angular.z

    def save_data(self):
        """Save image and twist data for later model training """
        # need to use full absolute path for some reason?
        fname = str("%s.png" % str(self.ts))

        # TODO: This can be improved
        db = {"ts": self.ts, "fname": fname, "speed": self.speed, "steer": self.steer}

        self.df = pd.concat(
            [self.df, pd.DataFrame.from_dict(db, orient="index").T], ignore_index=True
        )

        # save image
        PIL.Image.fromarray(self.img).save(self.SAVE_DIR + fname)

        # TODO: save every 20 increments? or so
        # TODO: just save the increment
        # save entire df (dumb, only need increment) (but can use ~ home shortcut here?)
        self.df.to_csv(self.SAVE_DIR + "data.csv")

        self.img_idx += 1

        # TODO: dont rewrite the whole csv on each run,
        # how to separate, into separate runs, or just load the csv initially?


if __name__ == "__main__":

    rospy.init_node("mini_imitation_learning")
    rospy.loginfo("Hello Imitation")

    imitiation = ImitationLearner()

    rospy.spin()


# TODO: i would love TwistStamped to be able to use message_filters
# TODO: TOO SLOW
# TODO: refactor model loading / preprocessing outside class

# RUn
# roslaunch control manual_control.launch
# roslaunch jetbot_ros jetbot_cam_compressed_one.launch
# rosrun imitation_learning imitation_learning.py

# roslaunch imitation_learning imitation_learning
