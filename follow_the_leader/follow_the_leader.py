#! /usr/bin/env/python3


import rospy
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist


class FollowTheLeader():
    """ Simple Follow the Leader demo """ 
    def __init__(self):

        self.linear_x = 0
        self.angular_z = 0

        self.twist_pub = rospy.Publisher("follow_vel", Twist, queue_size=10)
        rospy.Subscriber("/detectnet/detections", Detection2DArray, self.detection_callback)


    def detection_callback(self, data):

        twist = Twist()

        STEER_INCREMENT = 0.01
        SPEED_INCREMENT = 0.01

        # print(data.detections)
        IMG_SHAPE = (1280, 720)
        X_MIN, X_MAX = IMG_SHAPE[0] * 0.25, IMG_SHAPE[0] * 0.75

        for det in data.detections:


            if det.results[0].id == 1:
                #id = 1: person
                print("\nPerson Detected!")

                # if person to left, turn left
                if det.bbox.center.x < X_MIN:
                    print("TURNING LEFT")
                    self.angular_z -= STEER_INCREMENT


                # if person to right, turn right
                if det.bbox.center.x > X_MAX:
                    print("TURNING RIGHT")
                    self.angular_z += STEER_INCREMENT


                # if person is small, speed up
                if det.bbox.size_x < 400:
                    print("MOVE FASTER")
                    self.linear_x += SPEED_INCREMENT


                # if person is big, slow down?
                if det.bbox.size_x > 600:
                    print("SLOW DOWN")
                    self.linear_x -= SPEED_INCREMENT
                


                # should use lidar to measure distance to lead?
                # rely on e brake to stop?

            else:
                rospy.loginfo("No Person Detected. Stopping.")

                self.linear_x -= 0.01

        self.linear_x = max(0, self.linear_x)
        twist.linear.x = self.linear_x
        twist.angular.z = self.angular_z

        # publish twist
        self.twist_pub.publish(twist)


if __name__ == "__main__":

    rospy.init_node("follow_the_leader")

    follow_the_leader = FollowTheLeader()

    rospy.spin()