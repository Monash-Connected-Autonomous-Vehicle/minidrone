import rclpy
import numpy as np
from sklearn.cluster import DBSCAN
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from lidar_occupancy.true_bresenham import true_bresenham

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
        self.declare_parameter('grid_width', 100)
        self.declare_parameter('scan_dim_factor', 10.0)  # Scan importance factor between angular and radial data, in rad/m
        
        # Important ROS objects
        self.cloud_sub = self.create_subscription(PointCloud2, '/velodyne_points', self.cloud_callback, 10)
        self.occ_pub = self.create_publisher(OccupancyGrid, 'grid_out', 10)

        # Other important objects
        self.scan = DBSCAN(eps=0.5,  # TODO: add these numbers as ros params
                           min_samples=5, 
                           metric=self.cluster_metric, 
                           algorithm='auto',  # What needs to change here?
                           leaf_size=10)

    def cluster_metric(self, p1, p2):
        """
        Custom distance metric between two 2D scanned points, with scanner at origin.
        """
        # Calculate angle difference and squared radii
        rr1, rr2 = p1[0]**2 + p1[1]**2, p1[0]**2 + p1[1]**2 
        dth = np.arccos((p1[0]*p2[0] + p1[1]*p2[1])/np.sqrt(rr1*rr2))
        if dth > np.pi: dth = 2*np.pi - dth
        # Calculate scan distance metric
        a = self.get_parameter('scan_dim_factor').get_parameter_value().double_value
        return np.sqrt(a**2*abs(rr1-rr2) + dth**2)
        

    def cloud_callback(self, msg):
        # TODO: Read cloud in to desired format and segment

        # TODO: Determine contiguity of points
        example_array = np.array([[1.0, 1.1],[2.0, -2.2]])
        labels = self.scan.fit(example_array).labels_
        

        pts = [np.array([[1.0, 0.0], [0.5, 4.0]])]  # let pts contain numpy arrays of sequential contiguous points

        grid_res = self.get_parameter('grid_resolution').get_parameter_value().double_value
        grid_w = self.get_parameter('grid_width').get_parameter_value().integer_value
        grid = np.zeros((grid_w, grid_w), dtype=int)
        for cluster in pts:
            # Convert points to nearest grid coordinates
            cluster = (cluster/grid_res + grid_w/2).astype(int)
            for i in range(len(cluster)-1):
                # Handle points outside of grid
                if not np.any(np.logical_or(cluster[i:i+1, :] < 0, cluster[i:i+1, :] > grid_w)):
                    for px, py in true_bresenham(cluster[i, :], cluster[i+1, :]):
                        grid[px, py] = 127  # Populate grid
        
        # Create occupancy grid object
        pub_time = self.get_clock().now().to_msg()
        occ = OccupancyGrid()
        occ.header.frame_id = 'velodyne'
        occ.header.stamp, occ.info.map_load_time = pub_time, pub_time
        occ.info.resolution = grid_res
        occ.info.height, occ.info.width = grid_w, grid_w
        occ.info.origin.position.x = -grid_res*grid_w/2
        occ.info.origin.position.y = -grid_res*grid_w/2
        occ.data = grid.flatten().tolist()

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