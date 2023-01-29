import rclpy
from rclpy.node import Node


class PinionControlNode(Node):
    def __init__(self):
        super.__init__('pinion_control_node')
        

def main(args=None):
    # Start
    rclpy.init(args=args)
    pinion_control_node = PinionControlNode()
    rclpy.spin(pinion_control_node)
    
    # Stop
    pinion_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()