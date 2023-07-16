import numpy as np
from nav2_msgs import *
from nav_msgs.msg import *
import rclpy
from rclpy.node import Node
import random
import cv2
import math
from datetime import datetime
# A simple node that creates a random occupancy grid of given size where approximately 20% of the cells are occuapied, then publishes the occupancy grid to a topic

class OG_Example(Node):

  def __init__(self):
        self = self
        
        self.occupancy_grid = OccupancyGrid()
        super().__init__('og_example')
        self.og_publisher = self.create_publisher(OccupancyGrid,'very_custom_occupancy_grid',1)
        timer_period = 1
        self.timer = self.create_timer(timer_period,self.Create_pre_defined_occupancy_grid)
        
  

  """def Create_random_og_data(self):

    dataCount = self.occupancy_grid.info.width*self.occupancy_grid.info.height
    dataCount = round(dataCount)
    ogData = np.zeros(dataCount)
    
    #self.get_logger().info('datacount = "%d"'%dataCount)
    

    
    #self.ocuupancy_grid.data.clear
    self.get_logger().info('data size: "%d"' % len(self.occupancy_grid.data))
    if len(self.occupancy_grid.data) ==0:

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
    
    
      self.occupancy_grid.data =ogData

      self.get_logger().info('data size: "%d"' % len(self.occupancy_grid.data))
  
  def Create_random_occupancy_grid(self):

    #self.occupancy_grid = OccupancyGrid()
    
    self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
    self.occupancy_grid.header.frame_id = "custom_occupancy_grid"

    self.occupancy_grid.info.resolution = 0.25
    self.occupancy_grid.info.width = 40
    self.occupancy_grid.info.height = 40

    self.occupancy_grid.info.origin.position.x = 5.0
    self.occupancy_grid.info.origin.position.y = 5.0
    self.occupancy_grid.info.origin.position.z = 0.0
    
    self.occupancy_grid.info.origin.orientation.x = 0.0
    self.occupancy_grid.info.origin.orientation.y = 0.0
    self.occupancy_grid.info.origin.orientation.z = 0.0
    self.occupancy_grid.info.origin.orientation.w = 0.0

    self.Create_random_og_data()
   

    og = self.occupancy_grid
    self.og_publisher.publish(og)
    #self.get_logger().info('Publishing: "%s"' % self.ocuupancy_grid.data)"""

  def Create_pre_defined_occupancy_grid(self):
    
    
    self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
    self.occupancy_grid.header.frame_id = "custom_occupancy_grid"

    
    

    self.occupancy_grid.info.origin.position.x = 10.0
    self.occupancy_grid.info.origin.position.y = 10.0
    self.occupancy_grid.info.origin.position.z = 0.0
    
    self.occupancy_grid.info.origin.orientation.x = 0.0
    self.occupancy_grid.info.origin.orientation.y = 0.0
    self.occupancy_grid.info.origin.orientation.z = 0.0
    self.occupancy_grid.info.origin.orientation.w = 0.0


    #self.occupancy_grid.info.width = 10
    #self.occupancy_grid.info.height = 10
    #self.occupancy_grid.info.resolution = 1.0
    
    """self.occupancy_grid.data = [
                                  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                                100,100,100,100,  0,  0,100,100,100,100,
                                100,100,100,  0,  0,  0,100,100,100,100,
                                100,100,100,  0,  0,100,100,100,100,100,
                                100,100,100,  0,  0,  0,100,100,100,100,
                                100,100,100,100,  0,  0,100,100,100,100,
                                100,100,100,100,  0,  0,  0,100,100,100,
                                100,100,100,100,100,  0,  0,100,100,100,
                                100,100,100,100,  0,  0,  0,100,100,100, 
                                  0,  0,  0,  0,  0,  0,  0,  0,  0,  0]"""


    path = r'/home/omar/images/loop3.png'
    image = cv2.imread(path)
    print(len(image[0]))
    l = len(self.occupancy_grid.data)
    self.get_logger().info('data length = "%d"'%l)
    og_height = len(image)
    og_width = len(image[0])
    dataCount = og_height*og_width 
    og_data = np.zeros(dataCount)

    start = datetime.now()

    for i  in range(dataCount):
      m = math.floor(i/og_width)
      n = i - m*og_width
      if sum(image[m][n]) == 255*3:
        og_data[i] = 0
      else:
        
        og_data[i] = 100  
    
    end = datetime.now()

    #print('conversion time:',(end-start).microseconds,' us')

    og_data = og_data.tolist()    
    og_data = [int(a) for a in og_data]
    print(len(og_data))
    #print(og_data)


    self.occupancy_grid.info.width = og_width
    self.occupancy_grid.info.height = og_height
    self.occupancy_grid.info.resolution = 10.0/og_height

  
   
    
    
    self.occupancy_grid.data =og_data

    '''
    data = og_data
    og_data = np.zeros((og_height,og_width))
    
    

    #start = datetime.now()
    

    for i in range(og_height):
      og_data[i] = data[i*og_height:(i+1)*og_height]

    dim = 1
    kernel = np.ones((dim,dim),np.uint8)
    dilation = cv2.dilate(og_data,kernel,iterations = 1)


    
    og_data = np.zeros((og_height*og_width))

    start = datetime.now()

    for i in range(og_height):
      og_data[i*og_height:(i+1)*og_height] = dilation[i]

    end = datetime.now()

    print('conversion time:',(end-start).microseconds,' us')

    og_data = og_data.tolist()    
    og_data = [int(a) for a in og_data]

    self.occupancy_grid.data =og_data

    #end = datetime.now()

    #print('conversion time:',(end-start).microseconds,' us')
    print (sum(og_data))'''
    og = self.occupancy_grid
    self.og_publisher.publish(og)


    return self.occupancy_grid
  


"""def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = OG_Example()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
  




  
      

  

