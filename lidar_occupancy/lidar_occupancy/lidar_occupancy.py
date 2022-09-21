import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid


class LidarOccupancyNode(Node):
    """
    Creates an occupancy grid based on instantaneous information from LiDAR pointcloud data.

    ...

    Parameters
    ----------
    None

    Topics
    ------
    cloud_in : L{sensor_msgs.PointCloud2}
        Subscribed pointcloud

    grid_out : L{nav_msgs.OccupancyGrid}
        Published occupancy grid

    """
    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Important ROS objects
        self.cloud_sub = self.create_subscription(PointCloud2, 'cloud_in', self.cloud_callback, 10)
        self.occ_pub = self.create_publisher(OccupancyGrid, 'grid_out', 10)

    def cloud_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    lidar_occupancy = LidarOccupancyNode()
    rclpy.spin(lidar_occupancy)
    lidar_occupancy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()