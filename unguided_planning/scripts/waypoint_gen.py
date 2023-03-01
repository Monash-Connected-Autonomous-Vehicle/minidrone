import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
#from mcav_msgs.msg import WaypointArray

from unguided_planning.trajectory_tools import sample_trajectory

class WaypointNode(Node):
    """
    Node that generates local waypoints in response to updates in detected lanes an lidar occupancy
    ...

    Parameters
    ----------

    Topics
    ------
    lidar_occupancy : L{nav_msgs.OccupancyGrid}
        Subscribed occupancy grid of obstacles detected via lidar

    lane_occupancy : L{nav_msgs.OccupancyGrid}
        Subscribed occupancy grid of lanes detected by camera and CV system
        
    waypoints : L{mcav_msg.WaypointArray}
        Published local waypoints generated upon update in lidar or lane occupancy 

    """
    def __init__(self):
        super().__init__('waypoint_node')

        # ROS2 Parameters
        self.declare_parameter('sample_grid_width', 10, 'Occupancy grid pixel distance between sampled points')

        # Important ROS objects
        self.lidar_sub = self.create_subscription(OccupancyGrid, 'lidar_occupancy', self.lidar_callback, 10)
        self.lane_sub = self.create_subscription(OccupancyGrid, 'lane_occupancy', self.lane_callback, 10)
        #self.pub = self.create_publisher(WaypointArray, 'waypoints', 10)

        # Important objects
        self.lidar_grid = None
        self.lane_grid = None
        self.grid = None

        self.erode_kernel = np.ones((5, 5), np.uint8)

    def _update_grid(self):
        # Naive grid overlaying
        self.grid = self.lane_grid if self.lidar_grid is None else \
                    self.lidar_grid if self.lane_grid is None else \
                    self.lidar_grid + self.lane_grid

    def lidar_callback(self, msg):
        self.lidar_grid = np.array(msg.data, dtype=bool).reshape((msg.info.width, msg.info.height))
        self._update_grid()


    def lane_callback(self, msg):
        self.lane_grid = np.array(msg.data, dtype=int).reshape((msg.info.width, msg.info.height))
        
        # Sample grid points
        # 

    def _evaluate_trajectory(self, traj):
        pass

    



def main(args=None):
    rclpy.init(args=args)
    waypoint_node = WaypointNode()
    rclpy.spin(waypoint_node)
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
