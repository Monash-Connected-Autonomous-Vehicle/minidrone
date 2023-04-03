#!/usr/bin/env python3

import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
#from mcav_msgs.msg import WaypointArray

from mcav_rosdoc.node_doc import mcav_node_doc
from unguided_planning.space_tools import OccSpace, AngularTrajectory


@mcav_node_doc
class WaypointNode(Node):
    """
    Node generating local waypoints in response to updates in detected lanes an lidar occupancy

    TODO: Detailed description
    
    ...

    Parameters
    ----------
    sample_grid_res : int
        Number of vertices along axis in downsampling grid mesh.

    Subscribes
    ----------
    lidar_occupancy : L{nav_msgs.OccupancyGrid}
        Occupancy grid of physical obstructions.

    lane_occupancy : L{nav_msgs.OccupancyGrid}
        Occupancy grid of detected lane lines.
        
    Publishes
    ---------
    waypoints : L{mcav_msg.WaypointArray}
        Local waypoint path, in agent frame.

    """
    def __init__(self):
        super().__init__('waypoint_node')

        # ROS2 Parameters
        self.declare_parameter('sample_grid_res', 10)

        # Important ROS objects
        self.lidar_sub = self.create_subscription(OccupancyGrid, 'lidar_occupancy',
                                                  self.lidar_callback, 10)
        self.lane_sub = self.create_subscription(OccupancyGrid, 'lane_occupancy',
                                                 self.lane_callback, 10)
        #self.pub = self.create_publisher(WaypointArray, 'waypoints', 10)

        # Important objects
        self.lidar_grid = None
        self.lane_grid = None
        self.grid = None

        self.erode_kernel = np.ones((5, 5), np.uint8)

    def _update_grid(self):
        '''Overlay recieved grids onto eachother'''
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
        ''''''
        pass


def main(args=None):
    rclpy.init(args=args)
    waypoint_node = WaypointNode()
    print(type(waypoint_node))
    rclpy.spin(waypoint_node)
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()