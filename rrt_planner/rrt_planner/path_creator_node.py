import rclpy
from rclpy.node import Node
from rrt_planner.utils.RRTC import RRTC
#from utils.RRTC import RRTC
from rrt_planner.utils.Polygon import Rectangle
import matplotlib.pyplot as plt
import numpy as np

class Creator(Node):

    def __init__(self):
        super().__init__('obstacle_creator')
        ''' 
        INPUTS
        '''

        self.obstacle_coords = [(2,2),(9.5,9.5)]
        self.goal = np.array([3.0, 4.0])
        self.start = np.array([0, 0])
        self.width = 10
        self.height = 10
        self.expand_dis = 0.5 
        self.path_resolution = 0.2
        self.iter = 500

        '''
        Planning steps:
            - generate obstacles
            - instantiate RRT class with set params for planning
            - plan optimum path with a set number of max iterations 
        '''
        self.obstacles = self.generate_obstacles(self.obstacle_coords) 
        self.rrtc = RRTC(start=self.start, goal=self.goal, width=self.width, height=self.height, obstacle_list=self.obstacles,expand_dis=self.expand_dis, path_resolution=self.path_resolution)
        self.optimum_path = self.generate_path(self.iter, self.rrtc)
    
    # Generate obstacles with accepted datatype for RRTC 
    def generate_obstacles(self,obstacles):

        all_obstacles = []

        for obstacle in obstacles:
            origin = np.array([obstacle[0],obstacle[1]])
            dim = 2 ## size of obstacles represented as squares
            all_obstacles.append(Rectangle(origin,dim,dim))

        return all_obstacles
    
    # Use L2 Norm to find best path 
    def generate_path(self,iter,RRT):
        paths = []
        costs = []

        for iter in range(iter):
            paths.append(RRT.planning())
            
        for path in paths:
            if path is not None:
                cost = np.linalg.norm(path)
                costs.append(float(cost))
                print(cost,len(path))
                cost = 0
        
        min_value = min(costs)
        min_index = costs.index(min_value)
        optimum = paths[min_index]

        try:
            self.visualise_path(optimum)
            return paths[min_index]
        except:
            print("Path could not be planned!")
    
    # Plot occupancy grid with path
    def visualise_path(self,optimum):
        plt.scatter(*zip(*optimum), label='Generated path')
        plt.scatter(*zip(*self.obstacle_coords),s=100,alpha=0.5,label='Obstacles')
        plt.scatter(self.start[0],self.start[1], s=100,alpha=0.7,label='Start')
        plt.scatter(self.goal[0],self.goal[1], s=100,alpha=0.9,label='Goal')
        plt.legend()
        plt.show()
        return

def main(args=None):
    rclpy.init(args=args)
    creator = Creator()
    rclpy.spin(creator)
    creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()