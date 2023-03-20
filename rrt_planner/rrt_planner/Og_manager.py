import numpy as np
from nav2_msgs import *
from nav_msgs.msg import *
import rclpy
from rclpy.node import Node
import random

# A simple node that creates a random occupancy grid of given size where approximately 20% of the cells are occuapied, then publishes the occupancy grid to a topic

class OG_Manager(Node):

  def __init__(self):
        self = self
        
        self.ocuupancy_grid = OccupancyGrid()
        super().__init__('og_publisher')
        self.og_publisher = self.create_publisher(OccupancyGrid,'custom_occupancy_grid',1)
        timer_period = 1
        self.timer = self.create_timer(timer_period,self.Create_pre_defined_occupancy_grid)
        
  

  def Create_random_og_data(self):

    dataCount = self.ocuupancy_grid.info.width*self.ocuupancy_grid.info.height
    dataCount = round(dataCount)
    ogData = np.zeros(dataCount)
    
    #self.get_logger().info('datacount = "%d"'%dataCount)
    

    
    #self.ocuupancy_grid.data.clear
    self.get_logger().info('data size: "%d"' % len(self.ocuupancy_grid.data))
    if len(self.ocuupancy_grid.data) ==0:

      for i in range(dataCount):
        self.get_logger().info('i = "%d"'%i)

        if random.randint(1,5) ==1 :
                ogData[i] = 100
                #self.ocuupancy_grid.data.append(100)
                
        else:
                ogData[i] = 0
                #self.ocuupancy_grid.data.append(0)

                
      
      ogData = ogData.tolist()    
      ogData = [int(a) for a in ogData]
    
    
      self.ocuupancy_grid.data =ogData

      self.get_logger().info('data size: "%d"' % len(self.ocuupancy_grid.data))
  
  def Create_pre_defined_occupancy_grid(self):

    #self.occupancy_grid = OccupancyGrid()
    
    self.ocuupancy_grid.header.stamp = self.get_clock().now().to_msg()
    self.ocuupancy_grid.header.frame_id = "custom_occupancy_grid"

    self.ocuupancy_grid.info.resolution = 0.25
    self.ocuupancy_grid.info.width = 40
    self.ocuupancy_grid.info.height = 40

    self.ocuupancy_grid.info.origin.position.x = 5.0
    self.ocuupancy_grid.info.origin.position.y = 5.0
    self.ocuupancy_grid.info.origin.position.z = 0.0
    
    self.ocuupancy_grid.info.origin.orientation.x = 0.0
    self.ocuupancy_grid.info.origin.orientation.y = 0.0
    self.ocuupancy_grid.info.origin.orientation.z = 0.0
    self.ocuupancy_grid.info.origin.orientation.w = 0.0

    self.Create_random_og_data()
   

    og = self.ocuupancy_grid
    self.og_publisher.publish(og)
    #self.get_logger().info('Publishing: "%s"' % self.ocuupancy_grid.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = OG_Manager()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

  




  
      

  

