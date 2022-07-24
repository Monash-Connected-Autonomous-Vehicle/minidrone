import rclpy
from rclpy.node import Node
import sys

from sensor_msgs.msg import PointCloud2
import clustering_2d_brake.cloud_reader as cloud_reader


class CloudTo2DNode(Node):
    """
    A node that isolates a range of z values within a pointcloud, and republishes them as projected to a 2D layer at the specified height
    
    ...

    Parameters
    ----------
    slice_height: double
        z-level to extract points of from input pointcloud, and to output points
    height_var: double
        z deviation from slice_height tolerated for points to be republished

    Topics
    ------
    cloud_in
        Pointcloud topic for the node to subscribe to
    layer_out
        Pointcloud topic for the node to publish the 2D layer to

    """
    def __init__(self):
        super().__init__('cloud_to_2d')
        self.declare_parameter('slice_height', 0.0)
        self.declare_parameter('height_var', 0.001)

        self.subscription_ = self.create_subscription(PointCloud2, 'cloud_in', self.cloud_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud2, 'layer_out', 10)

    def cloud_callback(self, msg):
        header = msg.header
        height = self.get_parameter('slice_height').get_parameter_value().double_value
        height_var = self.get_parameter('height_var').get_parameter_value().double_value
        points = [(p[0], p[1], height) for p in cloud_reader.read_pointcloud_slice(msg, height, height_var)]
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