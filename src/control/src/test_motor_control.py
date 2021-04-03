#!/usr/bin/env python3

import time
import rospy
import numpy as np

from geometry_msgs.msg import TwistStamped

""" Test script to cycle through running each of the actuators """

def main():

    # ROS setup
    rospy.init_node("minidrone_motor_control_test_node")
    
    rospy.loginfo("-------------- Node Initialised --------------")
    rospy.loginfo("MiniDrone Motor Control Test Script")

    # publisher
    pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=10)

    # run static tests
    run_static_test(pub)
    
    # chicane test
    run_chicane_test(pub)

    # finish test
    run_test_cleanup(pub)

def run_static_test(pub):
    
    TEST_TIME = 2
    idx = 0

    # testing setup
    test_values = [(1.0, 0), (0.0, 0.5), (0.0, -0.5), (0.5, 0.5), (0.5, -0.5), (0.0, 0.0)]
    test_desc = ["Forward", "Left", "Right", "Forward Left", "Forward Right", "Stationary"]

    rospy.loginfo("-------------- Beginning Static Test --------------")

    while not rospy.is_shutdown():
        
        # get test values
        x, z = test_values[idx]
        desc = test_desc[idx]

        # twist message
        twist = TwistStamped()
        twist.twist.linear.x = x
        twist.twist.angular.z = z

        rospy.loginfo(f"Testing: {desc}")
        time_start = time.time()

        while time.time() - time_start <= TEST_TIME:
            
            # update timestamp
            twist.header.stamp = rospy.Time.now()
            
            # publish twist message
            pub.publish(twist)

            # rospy.loginfo(f"Time Since: {time.time() - time_start:.2f}")
            #rospy.loginfo(f"Publishing Twist: {twist}")
            
            # sleep for a bit
            rospy.sleep(0.1)
        
        # increment test
        idx+=1

        if idx == len(test_values):
            rospy.loginfo("-------------- Finishing Static Test --------------")

            break


def run_test_cleanup(pub):
    """Return vehicle to neutral position"""

    # return to stationary and straight alignment
    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.twist.linear.x = 0.0
    twist.twist.angular.z = 0.0

    # publish
    pub.publish(twist)
    rospy.sleep(0.5)

    rospy.loginfo("-------------- Finishing Test --------------")


def run_chicane_test(pub):
    """ Run a chicane test, move the wheels from left to right and back while driving """
    
    rospy.loginfo("-------------- Starting Chicane Test --------------")

    # chicane test values
    r_max, l_max = 0.5, -0.5
    z_vals = np.concatenate((np.linspace(l_max, r_max, 100), 
                            np.linspace(r_max, l_max, 100)), axis=0)
    x_vals = np.ones_like(z_vals) * 0.5

    # descriptions
    time_start = time.time()
    desc = "Chicane"

    rospy.loginfo(f"Testing: {desc}")

    for x, z in zip(x_vals, z_vals):
        
        # twist message
        twist = TwistStamped()
        twist.twist.linear.x = x
        twist.twist.angular.z = z
    
        # update timestamp
        twist.header.stamp = rospy.Time.now()
        
        # publish twist message
        pub.publish(twist)

        
        # rospy.loginfo(f"Time Since: {time.time() - time_start:.2f}")
        #rospy.loginfo(f"Publishing Twist: {twist}")
        
        # sleep for a bit
        rospy.sleep(0.1)
    
    rospy.loginfo("-------------- Finishing Chicane Test --------------")



if __name__ == "__main__":

    main()