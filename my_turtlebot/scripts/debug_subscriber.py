#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2


class DebugSubscriber(Node):

    def __init__(self, topic_name):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(PointCloud2, topic_name + '/repub', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Data republished')
        self.get_logger().info('I heard: "%s", "%s"' % (msg.header, msg.width))
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    debug_subscriber = DebugSubscriber('/gpu_ray/pointcloud2')

    rclpy.spin(debug_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    debug_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()