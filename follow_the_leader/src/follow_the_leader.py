#!/usr/bin/env python3


import cv_bridge
import numpy as np
import PIL
import rospy
from geometry_msgs.msg import Twist
from motpy import Detection, MultiObjectTracker
from PIL import Image, ImageDraw
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray

# TODO: tune values for following
# TODO: remove video output from roslaunch 
# TODO: create better launch file
# TODO: clean up console output



class FollowTheLeader():
    """ Simple Follow the Leader demo """ 
    def __init__(self):

        self.linear_x = 0
        self.angular_z = 0

        self.tracks = None
        self.tracked_objects = {}

        self.bridge = cv_bridge.CvBridge()
        self.twist_pub = rospy.Publisher("follow_vel", Twist, queue_size=10)
        rospy.Subscriber("/detectnet/detections", Detection2DArray, self.detection_callback)
        fps=10
        self.tracker = MultiObjectTracker(dt=1/fps) # assume 10fps?

        cam_topic = "video_source/compressed"
        # self.cam_sub = rospy.Subscriber(cam_topic, CompressedImage, self.img_callback, queue_size=1)
        self.track_img_pub = rospy.Publisher("/tracked_img/compressed", CompressedImage, queue_size=1)



    def detection_callback(self, data):

        # format detection bounding boxes

        detection_boxes = []

        # format detections for tracker
        for det in data.detections:

            label = det.results[0].id
            score = det.results[0].score

            if label == 1:
                x0 = det.bbox.center.x - det.bbox.size_x / 2
                x1 = det.bbox.center.x + det.bbox.size_x / 2                        
                y0 = det.bbox.center.y - det.bbox.size_y / 2
                y1 = det.bbox.center.y + det.bbox.size_y / 2

                box = [x0, y0, x1, y1]

                detection_boxes.append(Detection(box=np.array(box), score=score))


        # # update tracker state with new detections
        self.tracker.step(detections=detection_boxes)
        
        # # get active tracks
        self.tracks = self.tracker.active_tracks()
        # print(self.tracks)

        # if self.tracks:
        IMG_SHAPE = (720, 1280, 3)
        img = np.zeros(shape=IMG_SHAPE,dtype=np.uint8)
        track_img = self.draw_tracks(img, self.tracks)
        self.publish_img(track_img, pub="track")

        # twist = Twist()

        # STEER_INCREMENT = 0.01
        # SPEED_INCREMENT = 0.01

        # # print(data.detections)
        # IMG_SHAPE = (1280, 720)
        # X_MIN, X_MAX = IMG_SHAPE[0] * 0.4, IMG_SHAPE[0] * 0.6
        # SIZE_MIN, SIZE_MAX = 500, 700

        # for det in data.detections:


        #     if det.results[0].id == 1:
        #         #id = 1: person
        #         rospy.loginfo("---------------- Person Detected! ----------------")

        #         # if person to left, turn left
        #         if det.bbox.center.x < X_MIN:
        #             print("Following Leader. Turning Left.")
        #             self.angular_z += STEER_INCREMENT


        #         # if person to right, turn right
        #         if det.bbox.center.x > X_MAX:
        #             rospy.loginfo("Following Leader. Turning Right.")
        #             self.angular_z -= STEER_INCREMENT


        #         # if person is small, speed up
        #         if det.bbox.size_x < SIZE_MIN:
        #             print("Leder too far. Speeding Up.")
        #             self.linear_x += SPEED_INCREMENT


        #         # if person is big, slow down?
        #         if det.bbox.size_x > SIZE_MAX:
        #             rospy.loginfo("Leader too close. Slowing Down.")
        #             self.linear_x -= SPEED_INCREMENT
                

        #         # TODO:
        #         # should use lidar to measure distance to lead?
        #         # rely on e brake to stop?

        #     else:
        #         rospy.loginfo("No Person Detected. Stopping.")

                # self.linear_x -= 0.01

        # self.linear_x = max(0, self.linear_x)
        # twist.linear.x = self.linear_x
        # twist.angular.z = self.angular_z

        # # publish twist
        # self.twist_pub.publish(twist)


    def draw_bounding_box(self, draw, box, color, rect=True, track=True, text=None):

        # calculate track centre
        centre_px = self.calculate_track_centre(box)

        if rect:
            # draw bounding box
            draw.rectangle(
                [(int(box[0]), int(box[1])), (int(box[2]), int(box[3]))], outline=color
            )
        if text:
            draw.text((int(box[0]), int(box[1]) - 10), text, align="left")

        if track:

            radius = 20
            draw.ellipse(
                (
                    centre_px[0] - radius,
                    centre_px[1] - radius,
                    centre_px[0] + radius,
                    centre_px[1] + radius,
                ),
                fill=color,
                outline=color,
            )

    def calculate_track_centre(self, box):
        w, h = (box[2] - box[0], box[3] - box[1])
        centre_px = box[0] + w / 2, box[1] + h / 2
        return centre_px

    def draw_tracks(self, img, tracks):

        # draw detections and tracks
        if isinstance(img, np.ndarray):
            img = PIL.Image.fromarray(img)
        draw = ImageDraw.Draw(img)

        for track in tracks:
            
            # relabel track id as label
            if track.id not in self.tracked_objects.keys():
                 self.tracked_objects[track.id] = len(self.tracked_objects)

            # draw bounding box
            label = f"person {self.tracked_objects[track.id]:02d}" 
            self.draw_bounding_box(draw, track.box, (0, 255, 0), rect=True, text=label)
            # TODO: distinguish class being tracked
            # TODO: dict for tracks / class
        track_img = np.array(img)
    
        return track_img

    def img_callback(self, msg):

        # img = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
        
        # read ros image as cv2 img
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img = img[:, :, ::-1] # bgr to rgb
        print("SHAPE: ", img.shape)
        if self.tracks:
            print(self.tracks)

            track_img = self.draw_tracks(img, self.tracks)
            self.publish_img(track_img, pub="track")


            
    def publish_img(self, img, pub):
    
        img = img[:, :, ::-1] # rgb to bgr (bridge is bgr)

        # republish image
        msg = self.bridge.cv2_to_compressed_imgmsg(img, "png")
        msg.header.stamp = rospy.Time.now()
        if pub=="track":
            self.track_img_pub.publish(msg)

if __name__ == "__main__":

    rospy.init_node("follow_the_leader")

    follow_the_leader = FollowTheLeader()

    rospy.spin()
