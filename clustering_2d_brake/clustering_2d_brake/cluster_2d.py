import rclpy
import numpy as np 
import cv2
import math

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import KMeans

import clustering_2d_brake.cloud_tools as cloud_tools


class Cluster2DNode(Node):
    """
    k-means initialized with naive occupancy grid and connected components
    """
    def __init__(self):
        super().__init__('cloud_to_2d')
        self.declare_parameter('cluster_range', 1.5)
        self.declare_parameter('grid_fidelity', 200)
        
        self.subscription_ = self.create_subscription(PointCloud2, 'cloud_2d', self.cloud_callback, 10)
        self.labeled_cloud_publisher_ = self.create_publisher(PointCloud2, 'labeled_cloud', 10)
        self.cluster_publisher_ = self.create_publisher(PointCloud2, 'clusters', 10)

    
    def cloud_callback(self, msg):
        header = msg.header
        cluster_range = self.get_parameter('cluster_range').get_parameter_value().double_value
        grid_fidelity = self.get_parameter('grid_fidelity').get_parameter_value().integer_value

        # Read pointcloud to naive occupancy grid/binary image
        grid, points, cloud_z = np.zeros((grid_fidelity, grid_fidelity), np.uint8), [], None
        pixel_w = 2*cluster_range/grid_fidelity
        for x, y, z in cloud_tools.read_points(msg):
            if cloud_z is None: cloud_z = z
            points.append((x, y))
            if abs(x) < cluster_range and abs(y) < cluster_range:
                grid[int((x + cluster_range) / pixel_w), int((y + cluster_range) / pixel_w)] = 255
        
        # Perform connectivity analysis
        n_labels, label_grid, _, pixel_centroids = cv2.connectedComponentsWithStats(grid, connectivity=8)
        init_centroids = [(px*pixel_w-cluster_range, py*pixel_w-cluster_range) for px, py in pixel_centroids[1:]]
        
        # Perform k-means clustering
        kmeans = KMeans(n_labels-1, init=init_centroids, n_init=1).fit(points)
        labels = kmeans.labels_
        
        # Republish labeled pointcloud
        labeled_points = ((p[0], p[1], cloud_z, labels[i]) for i, p in enumerate(points))
        labeled_cloud = cloud_tools.create_cloud(header, ['x', 'y', 'z', 'label'], labeled_points)  # TODO: figure out why this doesn't work
        self.labeled_cloud_publisher_.publish(labeled_cloud)

        # TODO: publish cluster centroids





def main(args=None):
    rclpy.init(args=args)
    cluster_2d_node = Cluster2DNode()

    rclpy.spin(cluster_2d_node)

    cluster_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()