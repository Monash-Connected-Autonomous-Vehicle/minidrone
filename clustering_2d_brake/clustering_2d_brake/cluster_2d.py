import sklearn
import rclpy
import numpy as np 
import cv2
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import KMeans, DBSCAN

import clustering_2d_brake.cloud_tools as cloud_tools


class Cluster2DNode(Node):
    """
    A node that performs clustering with DBSCAN, and publishes a brake signal if any cluster is within a certain radius

    ...

    Parameters
    ----------
    brake_distance: double
        radius to send brake signal when violated by a cluster

    Topics
    ------
    cloud_2d:
        Input pointcloud where z values are assumed to be equal throughout
    labeled_cloud:
        Published copy of cloud_2d with outlier points removed and clustering
    brake:
        Publishes an empty twist message when an obstacle is detected

    """
    def __init__(self):
        super().__init__('cloud_to_2d')
        self.declare_parameter('brake_distance', 1.0)
        
        self.subscription_ = self.create_subscription(PointCloud2, 'cloud_2d', self.cloud_callback, 10)
        self.labeled_cloud_publisher_ = self.create_publisher(PointCloud2, 'labeled_cloud', 10)
        self.brake_publisher_ = self.create_publisher(Twist, 'brake', 10)  # TODO: actual relevant topic

    
    def cloud_callback(self, msg):
        header = msg.header

        # Read points and perform DBSCAN
        points, cloud_z = [], None
        for x, y, z in cloud_tools.read_points(msg):
            if cloud_z is None: cloud_z = z
            points.append([x, y])
        points = np.array(points).reshape(-1, 2)
        dbscan = DBSCAN().fit(points)
        labels = dbscan.labels_
        
        # Republish labeled pointcloud without outliers
        labeled_points = [(p[0], p[1], cloud_z, labels[i]) for i, p in enumerate(points) if labels[i] != -1]
        labeled_cloud = cloud_tools.create_cloud_from_list(header, labeled_points, ['x', 'y', 'z', 'label'])
        self.labeled_cloud_publisher_.publish(labeled_cloud)

        # Publish brake if any of the inlier points are close (assumes robot is at origin)
        d = self.get_parameter('brake_distance').get_parameter_value().double_value
        for p in labeled_points:
            if p[0]**2 + p[1]**2 < d**2:
                brake_msg = Twist()
                self.brake_publisher_.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)
    cluster_2d_node = Cluster2DNode()

    rclpy.spin(cluster_2d_node)

    cluster_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()