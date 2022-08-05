import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan

import clustering_2d_brake.cloud_tools as cloud_tools



class ScanToCloudNode(Node):
    """
    A node that subscribes to a laser scan topic and transcribes to a point cloud
    ...

    Parameters
    ----------
    scan_z: double
        z position of laser scan, default 0.0

    Topics
    ------
    scan:
        Input laser scan message
    cloud_2d:
        Output pointcloud

    """
    def __init__(self):
        super().__init__('scan_to_cloud')
        self.declare_parameter('scan_z', 1.0)
        
        self.subscription_ = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud2, 'cloud_2d', 10)
    
    def scan_callback(self, scan):
        z = self.get_parameter('scan_z').get_parameter_value().double_value
        ang, inc = scan.angle_min, scan.angle_increment
        points = [[dist*math.cos(ang+i*inc), dist*math.sin(ang+i*inc), z] for i, dist in enumerate(scan.ranges)]
        cloud = cloud_tools.create_cloud_from_list(scan.header, points)
        self.publisher_.publish(cloud)



def main(args=None):
    rclpy.init(args=args)
    scan_to_cloud_node = ScanToCloudNode()

    rclpy.spin(scan_to_cloud_node)

    scan_to_cloud_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()