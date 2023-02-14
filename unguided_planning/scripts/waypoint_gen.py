import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
#from mcav_msgs.msg import WaypointArray

from unguided_planning.trajectory_tools import sample_trajectory

class WaypointNode(Node):
    """
    TODO
    ...

    Parameters
    ----------
    add_out : int
        Integer added to the input when published to output topic

    Topics
    ------
    example_input : L{std_msgs.Int8}
        Subscribed integer input

    example_output : L{std_msgs.Int8}
        Published output upon recieving input, equal to example_input + add_out

    """
    def __init__(self):
        super().__init__('waypoint_node')

        # ROS2 Parameters

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
        self.lane_grid = np.array(msg.data, dtype=bool).reshape((msg.info.width, msg.info.height))
        self._update_grid()

    def _evaluate_trajectory(self, pts):
        # TODO: obtain mean/stdv square difference between closest left and right lane point to sample point
        thin_lanes = cv2.erode(self.lane_grid, self.erode_kernel, iterations=1)

    



def main(args=None):
    rclpy.init(args=args)
    waypoint_node = WaypointNode()
    rclpy.spin(waypoint_node)
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()