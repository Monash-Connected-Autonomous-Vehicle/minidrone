import rclpy
from rclpy.node import Node
import sys

from sensor_msgs.msg import PointCloud2
#from sensor_msgs_py.point_cloud2 import read_points  # THIS WOULD BE REALY NICE TO HAVE, WAS IMPLEMENTED IN GALACTIC

import clustering_2d_brake.cloud_reader as cloud_reader


class CloudTo2DNode(Node):
    """
    A node that isolates/publishes a horizontal layer of a subscribed pointcloud
    
    ...

    Parameters
    ----------
    layer_height : TODO

    Topics
    ------
    cloud_in
        Pointcloud topic for the node to subscribe to
    layer_scan
        Pointcloud topic for the node to publish to

    """
    def __init__(self):
        super().__init__('cloud_to_2d')
        self.declare_parameter('layer_height', '0')

        self.subscription_ = self.create_subscription(PointCloud2, 'cloud_in', self.cloud_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud2, 'layer_out', 10)

    def cloud_callback(self, msg):
        header = msg.header
        height = self.get_parameter('layer_height').get_parameter_value().integer_value  # Integer?
        print(msg)
        points = list(iter(cloud_reader.read_pointcloud_layer(msg, height)))
        out = cloud_reader.create_cloud_xyz32(header, points)
        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    cloud_to_2d_node = CloudTo2DNode()

    rclpy.spin(cloud_to_2d_node)

    cloud_to_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()