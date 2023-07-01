#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from glob_planner.RRT import RRTC
from matplotlib.ticker import MultipleLocator
from glob_planner.og_example import OG_Example as OG
from glob_planner.Polygon import Rectangle
from math import hypot as hypot
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import cv2

import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import OccupancyGrid
from datetime import datetime


class Creator(Node):

    def __init__(self):
        super().__init__('obstacle_creator')
        ''' 
        INPUTS
        '''

        #self.obstacle_coords = [(2,2),(9.5,9.5)]
        self.goal = np.array([4, 5])
        self.start = np.array([1, 9])
        self.width = 10
        self.height = 10
        self.expand_dis = 2 
        self.path_resolution = 0.2
        self.iter = 100
        self.rviz2_path = Path()
        ##self.path_publisher = self.create_publisher(Path,'rviz2_path',1)
        self.path_publisher = self.create_publisher(Path,'custom_occupancy_grid',1)
 

        '''
        Planning steps:
            - generate obstacles
            - instantiate RRT class with set params for planning
            - plan optimum path with a set number of max iterations 
        '''

        #Import the occupancy grid
        self.og = OG()
        self.Occupancy_grid = self.og.Create_pre_defined_occupancy_grid()
        
        #Import necessary parameters from said occupancy grid
        og_height =  self.Occupancy_grid.info.height
        og_width = self.Occupancy_grid.info.width
        og_res = self.Occupancy_grid.info.resolution
        og_data= self.Occupancy_grid.data

        #Dilate the occupancy grid data
        start = datetime.now()

        og_data_mat = np.zeros((og_height,og_width))
        for i in range(og_height):
            og_data_mat[i] = og_data[i*og_height:(i+1)*og_height]
        dim = 5
        kernel = np.ones((dim,dim),np.uint8)
        new_og_data = cv2.dilate(og_data_mat,kernel,iterations = 1)
        end = datetime.now()
        print('conversion time:',(end-start).microseconds,' us')



        self.rrtc = RRTC(start=self.start, goal=self.goal, width=self.width, height=self.height, og_height= og_height, og_width=og_width,
                        og_data= new_og_data,expand_dis=self.expand_dis,path_resolution=self.path_resolution,Calling_node=self, og_resolution=og_res)
        
        self.optimum_path = self.generate_path(self.iter, self.rrtc)

        self.rviz2_path.header.frame_id = "rviz2_path"
        self.rviz2_path.header.stamp =  self.get_clock().now().to_msg()
        self.rviz2_path_visualize(optimum=self.optimum_path)
        

        
        """
        self.obstacles = self.generate_obstacles(self.obstacle_coords) 
        self.rrtc = RRTC(start=self.start, goal=self.goal, width=self.width, height=self.height, obstacle_list=self.obstacles,expand_dis=self.expand_dis, path_resolution=self.path_resolution)
        self.optimum_path = self.generate_path(self.iter, self.rrtc)
        """
    
    # Use L2 Norm to find best path 
    def generate_path(self,iters,RRT):
        paths = []
        costs = []
        cost = 0
        startTime = datetime.now()
        for iter in range(iters):
            paths.append(RRT.planning())
            
        for path in paths:
            if path is not None:
                """cost = np.linalg.norm(path)
                costs.append(float(cost))
                print(cost,len(path))
                cost = 0"""
                for index in range(len(path)-1):
                    dx = path[index][0] - path[index+1][0]
                    dy = path[index][1] - path[index+1][1]
                    cost+= hypot(dx,dy)
                costs.append(cost)
        min_value = min(costs)
        min_index = costs.index(min_value)
        optimum = paths[min_index]
        endTime = datetime.now()
        print('path planning time',(endTime-startTime).microseconds,' us')

        try:
            self.visualise_path(optimum)
            return paths[min_index]
        except:
            print("Path could not be planned!")
    
    # Plot occupancy grid with path
    def visualise_path(self,optimum):
        
        xcoords = []
        ycoords = []
        for node in optimum:
            xcoords.append(node[0])
            ycoords.append(node[1])
        
        fig = plt.figure(figsize=(8,6))
        axes = plt.subplot(1, 1, 1)
        axes.axis([-1 ,11 ,-1 ,11])
        loc = MultipleLocator(1)
        axes.xaxis.set_major_locator(loc)
        axes.yaxis.set_major_locator(loc)

        plt.scatter(xcoords,ycoords,s=25,alpha=0.7, label='Generated path')
        plt.plot(xcoords,ycoords)
        plt.scatter(self.start[0],self.start[1], s=25,alpha=0.7,label='Start')
        plt.scatter(self.goal[0],self.goal[1], s=25,alpha=0.9,label='Goal')
        plt.legend()
        plt.xlim([-1,11])
        plt.ylim([-1,11])
        plt.grid()

        plt.show()

        

        return
    def rviz2_path_visualize(self,optimum):
        data = []
        """
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "rviz2_pose"
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        """
        for node in optimum:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "rviz2_pose"
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.position.x = float(node[0])
            pose.pose.position.y = float(node[1])

            print(pose.pose.position.x,",",pose.pose.position.y)
            
            data.append(pose)

        
        
        self.rviz2_path.poses = data
        
        self.path_publisher.publish(self.rviz2_path)

        return
            

def main(args=None):
    rclpy.init(args=args)
    creator = Creator()
    rclpy.spin(creator)
    creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
