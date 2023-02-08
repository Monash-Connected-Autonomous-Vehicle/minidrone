import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from mcav_msgs.msg import WaypointArray

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
        self.pub = self.create_publisher(WaypointArray, 'waypoints', 10)

        # Important objects
        self.lidar_grid = None
        self.lane_grid = None
        self.grid = None

    def _update_grid(self):
        # TODO: Layer lidar_gird and lane_grid onto eachother to make grid
        pass

    def lidar_callback(self, msg):
        # TODO: convent to numpy and save to lidar_grid
        self._update_grid()

    def lane_callback(self, msg):
        # TODO: convent to numpy and save to lane_grid
        self._update_grid()



def main(args=None):
    rclpy.init(args=args)
    waypoint_node = WaypointNode()
    rclpy.spin(waypoint_node)
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()