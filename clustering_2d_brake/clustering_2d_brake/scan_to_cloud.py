from numpy import NaN
import rclpy
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
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
        self.declare_parameter('min_dist', 0.05)
        self.declare_parameter('max_dist', 2.0)

        # QoS settings for subscriber
        sub_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            #history=QoSHistoryPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
        
        self.subscription_ = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=sub_profile)
        self.publisher_ = self.create_publisher(PointCloud2, 'cloud_2d', 10)
    
    def scan_callback(self, scan):
        z = self.get_parameter('scan_z').get_parameter_value().double_value
        min_d = self.get_parameter('min_dist').get_parameter_value().double_value
        max_d = self.get_parameter('max_dist').get_parameter_value().double_value
        
        ang, inc = scan.angle_min, scan.angle_increment
        points = [[dist*math.cos(ang+i*inc), dist*math.sin(ang+i*inc), z] 
                    for i, dist in enumerate(scan.ranges) if (dist > min_d) and (dist < max_d) and dist is not NaN]
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