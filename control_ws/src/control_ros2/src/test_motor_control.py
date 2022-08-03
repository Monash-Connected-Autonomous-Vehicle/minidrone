#!/usr/bin/env python3

import time
import rclpy
import numpy as np
import sys

from geometry_msgs.msg import TwistStamped

""" Test script to cycle through running each of the actuators """


def main():

    # ROS setup
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("minidrone_motor_control_test_node")
    node.get_logger().info("-------------- Node Initialised --------------")
    node.get_logger().info("MiniDrone Motor Control Test Script")

    # publisher
    pub = node.create_publisher(TwistStamped, "/cmd_vel", 10)

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
    test_values = [(0.2, 0), (0.0, 0.5), (0.0, -0.5),
                   (0.2, 0.5), (0.2, -0.5), (0.0, 0.0)]
    test_desc = ["Forward", "Left", "Right",
                 "Forward Left", "Forward Right", "Stationary"]
    self.get_logger().info("-------------- Beginning Static Test --------------")

    while not rclpy.is_shutdown():

        # get test values
        x, z = test_values[idx]
        desc = test_desc[idx]

        # twist message
        twist = TwistStamped()
        twist.twist.linear.x = x
        twist.twist.angular.z = z

        pub.get_logger().info(f"Testing: {desc}")
        time_start = time.time()

        while time.time() - time_start <= TEST_TIME:

            # update timestamp
            twist.header.stamp = rclpy.Time.now()

            # publish twist message
            pub.publish(twist)

            # rclpy.loginfo(f"Time Since: {time.time() - time_start:.2f}")
            #rclpy.loginfo(f"Publishing Twist: {twist}")

            # sleep for a bit
            rclpy.sleep(0.1)

        # increment test
        idx += 1

        if idx == len(test_values):
            pub.get_logger().info(
                "-------------- Finishing Static Test --------------")

            break


def run_test_cleanup(pub):
    """Return vehicle to neutral position"""

    # return to stationary and straight alignment
    twist = TwistStamped()
    twist.header.stamp = rclpy.Time.now()
    twist.twist.linear.x = 0.0
    twist.twist.angular.z = 0.0

    # publish
    pub.publish(twist)
    rclpy.sleep(0.5)

    node.get_logger().info("-------------- Finishing Test --------------")


def run_chicane_test(pub):
    """ Run a chicane test, move the wheels from left to right and back while driving """

    node.get_logger().info("-------------- Starting Chicane Test --------------")

    # chicane test values
    r_max, l_max = 0.5, -0.5
    z_vals = np.concatenate((np.linspace(l_max, r_max, 100),
                            np.linspace(r_max, l_max, 100)), axis=0)
    x_vals = np.ones_like(z_vals) * 0.2

    # descriptions
    time_start = time.time()
    desc = "Chicane"

    node.get_logger().info(f"Testing: {desc}")

    for x, z in zip(x_vals, z_vals):

        # twist message
        twist = TwistStamped()
        twist.twist.linear.x = x
        twist.twist.angular.z = z

        # update timestamp
        twist.header.stamp = rclpy.Time.now()

        # publish twist message
        pub.publish(twist)

        # rclpy.loginfo(f"Time Since: {time.time() - time_start:.2f}")
        #rclpy.loginfo(f"Publishing Twist: {twist}")

        # sleep for a bit
        rclpy.sleep(0.1)

    node.get_logger().info("-------------- Finishing Chicane Test --------------")


if __name__ == "__main__":

    main()
