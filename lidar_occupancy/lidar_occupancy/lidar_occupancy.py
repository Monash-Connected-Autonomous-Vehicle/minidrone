import rclpy
import numpy as np
from math import sqrt, acos
from sklearn.cluster import DBSCAN
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData

from lidar_occupancy.true_bresenham import true_bresenham
from lidar_occupancy.pcl2_serialization import read_points, read_points_numpy

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException


class LidarOccupancyNode(Node):
    """
    Creates an occupancy grid based on instantaneous information from LiDAR pointcloud data.

    ...

    Parameters
    ----------
    grid_resolution : float
        width of a pixel in the output occupancy grid, in meters

    grid_width : int
        with of the output occupancy grid, in pixels

    Topics
    ------
    cloud_in : L{sensor_msgs.PointCloud2}
        Subscribed pointcloud

    grid_out : L{nav_msgs.OccupancyGrid}
        Published occupancy grid

    """

    def __init__(self):
        super().__init__('lidar_occupancy')

        # ROS parameters
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('grid_width', 500)
        self.declare_parameter('scan_dim_factor', 0.2)  # Scan importance factor between angular and radial data

        # Important ROS objects
        self.cloud_sub = self.create_subscription(PointCloud2, '/velodyne_points', self.cloud_callback, 10)
        self.occ_pub = self.create_publisher(OccupancyGrid, 'grid_out', 10)

        # Other important objects
        self.scan = DBSCAN(eps=0.05, min_samples=4, leaf_size=3)

        # TODO Transformation bewteen velodyne frame and base frame for finding z_slice
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)


    def cloud_callback(self, msg):
        # TODO z_slice = velodyne z - base_footprint z = 0.523 from tf data
        # try:
        #     tf = self.buffer.lookup_transform('velodyne', 'base_footprint', rclpy.time.Time())
        # except TransformException:
        #     pass
        # print(tf)

        # Read in relevant points as cartesian and polar
        cartesian_pts = np.array([[x, y, z] for x, y, z, _, _, _ in read_points(msg) if self.point_filter([x, y, z])])[:, :2]
        polar_pts = np.array([polar_coord(p[0], p[1]) for p in cartesian_pts])
        # Scale the radial dimension, as a better distance metric for DBSCAN
        polar_pts[:, 0] *= self.get_parameter('scan_dim_factor').get_parameter_value().double_value

        # Order points based on dbscan labels
        labels = self.scan.fit(polar_pts).labels_
        sort_ind = np.argsort(labels)
        sorted_data, sorted_labels = np.concatenate((cartesian_pts, polar_pts[:, 1:]), axis=1)[sort_ind][1:], labels[sort_ind]
        point_clusters = np.split(sorted_data, np.where(sorted_labels[:-1] != sorted_labels[1:])[0])
        if sorted_labels[0] == -1: point_clusters = point_clusters[1:]
        # point_clusters should be a list containing arrays of contiguous points in cartesian, with their polar angle attached

        # Convert clustered points into occupancy grid with interpolation
        grid_res = self.get_parameter('grid_resolution').get_parameter_value().double_value
        grid_w = self.get_parameter('grid_width').get_parameter_value().integer_value
        grid = np.zeros((grid_w, grid_w), dtype=int)
        for cluster in point_clusters:
            # Sort and convert points to nearest grid coordinates, as sorted within a cluster by their theta value
            cluster = (cluster[cluster[:, 2].argsort(), :2]/grid_res + grid_w/2).astype(int)
            for i in range(len(cluster) - 1):
                # Handle points outside of grid
                if not np.any(np.logical_or(cluster[i:i+2, :] < 0, cluster[i:i+2, :] >= grid_w)):
                    for px, py in true_bresenham(cluster[i, :], cluster[i + 1, :]):
                        grid[py, px] = 127  # Populate grid

        # Create occupancy grid object
        pub_time = self.get_clock().now().to_msg()
        occ = OccupancyGrid()
        occ.header.frame_id = 'velodyne'
        occ.header.stamp, occ.info.map_load_time = pub_time, pub_time
        occ.info.resolution = grid_res
        occ.info.height, occ.info.width = grid_w, grid_w
        occ.info.origin.position.x = -grid_res * grid_w / 2
        occ.info.origin.position.y = -grid_res * grid_w / 2
        occ.data = grid.flatten().tolist()

        # Publish grid
        self.occ_pub.publish(occ)

    def point_filter(self, p: tuple[int]) -> bool:
        """ Filters a point if it should be kept or not

        Returns true for points worth considering e.g. ignores points that are part of the drone itself
        and ignores points below ground or above drone

        Parameters
        ----------
        p : tuple of int
            3-dimensional cartesian point

        Returns
        -------
        bool
            indication of if point should be kept or not
        """
        # TODO is a static method atm, slice values derived from lino-robot macros + transform info
        x_slice, y_slice, z_slice = 0.55, 0.62, 0.52

        # if within the area occupied by minidrone, ignore point
        if -x_slice <= p[0] <= x_slice and -y_slice <= p[1] <= y_slice:
            return False
        # if above ground, don't ignore point
        if -z_slice < p[2]:
            return True
        return False


def polar_coord(x: int, y: int) -> tuple[int]:
    """ Converts a pair of cartesian coordinates into polar coordinate form

    Takes a pair of coordinates x and y, and converts them into the form (r, theta)
    as polar coordinates

    Parameters
    ----------
    x : int
        cartesian x-coordinate of point
    y : int
        cartesian y-coordinate of point

    Returns
    -------
    tuple of int
        polar coordinate equivalent of input coordinates

    """
    r = sqrt(x ** 2 + y ** 2)

    if r == 0:
        coords = (r, None)
    else:
        coords = (r, -acos(x / r)) if y < 0 else (r, acos(x / r))

    return coords


def main(args=None):
    rclpy.init(args=args)
    lidar_occupancy = LidarOccupancyNode()
    rclpy.spin(lidar_occupancy)
    lidar_occupancy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
