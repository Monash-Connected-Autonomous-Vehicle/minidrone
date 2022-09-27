import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from lidar_occupancy.lidar_occupancy import true_bresenham

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
        super().__init__('minimal_publisher')

        # ROS parameters
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('grid width', 100)
        
        # Important ROS objects
        self.cloud_sub = self.create_subscription(PointCloud2, 'cloud_in', self.cloud_callback, 10)
        self.occ_pub = self.create_publisher(OccupancyGrid, 'grid_out', 10)

    def cloud_callback(self, msg):
        # TODO: Read cloud in to desired format and segment
        # TODO: Determine contiguity of points
        pts = []  # let pts contain numpy arrays of sequential contiguous points

        grid_res = self.get_parameter('grid_resolution').get_parameter_value().double_value
        grid_w = self.get_parameter('grid_width').get_parameter_value().integer_value
        grid = np.zeros((grid_w, grid_w), dtype=bool)
        for cluster in pts:
            # Convert points to nearest grid coordinates
            cluster = (cluster/grid_res + grid_w/2).astype(int)
            for i in range(len(cluster)-1):
                # Handle points outside of grid
                if not np.any(np.logical_or(cluster[i:i+1, :] < 0, cluster[i:i+1, :] > grid_w)):
                    for px, py in true_bresenham(cluster[i, :], cluster[i+1, :]):
                        grid[px, py] = True  # Populate grid
        
        # Create occupancy grid object
        meta = MapMetaData()
        meta.time, meta.resolution = self.get_clock().now(), grid_res
        meta.height, meta.width = grid_w, grid_w
        occ = OccupancyGrid()
        occ.info = meta
        occ.data = grid.tolist()

        # Publish grid
        self.occ_pub.publish(occ)


def main(args=None):
    rclpy.init(args=args)
    lidar_occupancy = LidarOccupancyNode()
    rclpy.spin(lidar_occupancy)
    lidar_occupancy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()