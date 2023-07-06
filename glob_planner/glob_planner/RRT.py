import numpy as np
import math 
from math import ceil, floor
import rclpy
from rclpy.node import Node


class RRTC:
    """
    A class that defines an instance of RRT path planning between two given global points


    Parameters
        ----------
        x : float
            the x coordinate of the node
        start : node
            the node that represnts the global starting point of the to-be-planned path
        end : node
            the node that represnts the global end point of the to-be-planned path
        map_width : float
            the width of the given occupancy grid in some arbirtrary units, that are the same as the units used for the start and end coordinates
        map_height : float
            the height of the given occupancy grid in some arbirtrary units, that are the same as the units used for the start and end coordinates
        og_width : int
            the number of squares the represent the width of the occupancy grid
        og_height: int
            the number of squares the represent the height of the occupancy grid
        og_resolution : float
            the resolution of the given occupancy grid, which is the ratio between map_width (or map_height) and og_width (or og_height)
        og_data : array (matrix og_height rows x og_width columns)
            the data of the occupancy grid, which used to determine the occupancy state (occupied or not occupied) of all the occupancy grid squares, a value of zero represent 
            a non-occupied square, the square is considered occupied for any other value
        expand_dis : float
            maximum distance between two subsequent nodes in the path (a node and its parent)
        path_resolution :  float

        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_nodes = max_points
        self.start_node_list = [] # Tree from start
        self.end_node_list = [] # Tree from end
        self.Calling_Node = Calling_node

    """

    class Node:
        """
        RRT Node, which essentially corresponds to a potential point in our final path

        Parameters
        ----------
        x : float
            the x coordinate of the node

        y : float
            the y coordinate of the node

        path_x: array of floats
            the x coordinates of the path that leads to the parent node, empty if parent is None

        path_y: array of floats
            the y coordinates of the path that leads to the parent node, empty if parent is None
        parent: Node or None
            The node closest to this node (self), none if there is no other node

        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start=np.zeros(2),
                 goal=np.array([120,90]),
                 width = 10,
                 height=10,
                 og_width = 10,
                 og_height = 10,
                 og_resolution = 1,
                 og_data = [],
                 expand_dis=1.0, 
                 path_resolution=0.5, 
                 max_points=150,
                 Calling_node= None):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        width, height: of the map the occuapncy grid will be fitted to
        og_height, og_width, og_resolution, og_data: height, width, resolution and data of the occupancy grid
        expand_dis: min distance between random node and closest node in rrt to it
        path_resolion: step size to considered when looking for node to expand
        
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.map_width = width
        self.map_height = height
        self.og_resolution = og_resolution
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_nodes = max_points
        self.og_data = og_data
        self.og_width = og_width
        self.og_height = og_height
        self.start_node_list = [] # Tree from start
        self.end_node_list = [] # Tree from end
        self.Calling_Node = Calling_node
        
    def planning(self):
        """
        rrt path planning
        """
        point_counter = 0
        path_counter = 0
        self.start_node_list = [self.start]
        end_node = self.end
        while len(self.start_node_list) <= self.max_nodes:
            rnd_node = self.get_random_node()
            expansion_ind = self.get_nearest_node_index(self.start_node_list,rnd_node)
            expansion_node = self.start_node_list[expansion_ind] ## closest node in tree to rnd_node

            nearby_node = self.steer(expansion_node,rnd_node,self.expand_dis) ## get node closer to rnd_node from expansion_node
            # add nearby_node to start_list if it is collision free 

            if self.new_node_is_free(new_node= nearby_node, og_width= self.og_width, og_resolution=self.og_resolution, og_data= self.og_data):
                if self.minor_path_is_collision_free(new_node= nearby_node,prev_node=expansion_node,og_resolution=self.og_resolution,og_width=self.og_width,og_data= self.og_data):
                    self.start_node_list.append(nearby_node)
                    #point_counter =0
                    #path_counter=0
                    
                '''else:
                    path_counter+=1
                    if(path_counter ==100):
                        print('100 paths failed')
                        p=1/0
                        
                            
                    
                    continue
            else:

                point_counter+=1
                if(point_counter ==100):
                    print('100 points failed')
                    p=1/0
                    
                        
                continue'''

            nearby_node = self.start_node_list[-1]
           

            d, thet = self.calc_distance_and_angle(end_node,nearby_node)

            # add the node that connects the trees and generate the path
            if d < self.expand_dis:
                end_node = self.steer(nearby_node,end_node,self.expand_dis)
                if self.minor_path_is_collision_free(new_node= nearby_node,prev_node=end_node,og_resolution=self.og_resolution,og_width=self.og_width,og_data= self.og_data):
                    
                    #print("destination reached")
                    self.start_node_list.append(end_node)
                
                    return self.generate_final_course(len(self.start_node_list) - 1, len(self.end_node_list) - 1)
        
    def steer(self, from_node, to_node, extend_length=float("inf")):
        """

        Given two nodes from_node, to_node, this method returns a node new_node such that new_node 
        is “closer” to to_node than from_node is.

        Parameters
        ----------
        from_node : Node
            The node we are starting the steering from

        to_node : Node
            The node we are steering to

        extend_length : float
            the maximum exapnd distance between two subsequent nodes in our path

        Returns
        -------
        new_node : Node
            the node that is within extend_length away from the from_node, and in the direction of to_node from from_node

        """

        #print("steering from: (",round(from_node.x,3),",",round(from_node.y,3),") to: (",round(to_node.x,3),",",round(to_node.y,3),")")
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        
        new_node.x += extend_length * cos_theta
        new_node.y += extend_length * sin_theta
        '''# How many intermediate positions are considered between from_node and to_node
        n_expand = math.floor(extend_length / self.path_resolution)

        # Compute all intermediate positions
        for _ in range(n_expand):
            new_node.x += self.path_resolution * cos_theta
            new_node.y += self.path_resolution * sin_theta
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)'''

        new_node.parent = from_node

        return new_node

   
  

    def new_node_is_free(self,new_node,og_width,og_resolution,og_data):
        """
        Determines if a given node lies in the free square on our occupancy grid
        """

        og_index = self.map_to_og_matrix_conversion(point=new_node,og_resolution=og_resolution)
       
        if og_data[og_index[0]][og_index[1]] ==0:
            
            #print('UNOCCUPIED POINT FOUND:',round(new_node.x,3),round(new_node.y,3))
            #print()
           
            
            return True
        else:
            
            #print('OCCUPIED POINT FOUND:',round(new_node.x,3),round(new_node.y,3))
            #print()
            return False 


    def map_to_og_matrix_conversion(self,point,og_resolution):
        """
        Returns the index of the square of the occuapncy grid a given point lies on in a matrix form in row-major form,
        so that it is easier to compare with [x,y] coordinates
        """
        position = np.array([point.x,point.y])
        ratio = 1/og_resolution

        og_column = floor(position[0]*ratio)
        og_row = floor(position[1]*ratio)
        
        og_index = [og_row,og_column]
        return og_index

    def og_matrix_to_map_conversion(self,og_grid,og_resolution):
        """
        returns the most bottom-left point in an occuapncy grid square, given its index in row-major form
        """

        ratio = 1/og_resolution
        x = (og_grid[1])/ratio
        y = (og_grid[0])/ratio
        return self.Node(x,y)

    def point_is_on_left_parameter_of_grid_square(self,point,og_resolution):
        """
        determines if a point lies perfectly on the vertical line between two occupancy gird squares
        """
        position = np.array([point.x,point.y])
        
        ratio = 1/og_resolution

        if floor(position[0]*ratio) == position[0]*ratio:
            return True
        else:
            return False

    def point_is_on_bottom_parameter_of_grid_square(self,point,og_resolution): ## this should be top not bottom
        """
        determines if a point lies perfectly on the horizontal line between two occupancy gird squares
        """
        position = np.array([point.x,point.y])
        
        ratio = 1/og_resolution

        if floor(position[1]*ratio) == position[1]*ratio:
            return True
        else:
            return False

    def minor_path_is_collision_free(self,new_node,prev_node,og_resolution,og_width,og_data):

        """
        
        determines if the straight line between two given points doesn't pass through any occupied squares
        """

        
        start = prev_node
        end = new_node
        
        ratio = 1/og_resolution
        
        ratio = 1/self.og_resolution
        starting_grid = self.map_to_og_matrix_conversion(point=start,og_resolution=og_resolution)
        end_grid = self.map_to_og_matrix_conversion(point=end,og_resolution=og_resolution)
        on_left = self.point_is_on_left_parameter_of_grid_square (point= start,og_resolution=og_resolution)
        on_bottom = self.point_is_on_bottom_parameter_of_grid_square(point= start,og_resolution=og_resolution)
        gradient = abs((end.y - start.y) / (end.x - start.x))
        delta_x = end.x - start.x
        delta_y = end.y-start.y
        

        counter = 0

        '''
        print('checking path between points:')
        print(round(start.x,3),round(start.y,3), 'and',round(end.x,3),round(end.y,3))
        print('GRADIENT IS:', round(gradient,2))
        '''
        #To decide the starting square of the occupancy grid if the point is between two grid squares
        if on_left or on_bottom:
            if delta_x <0 and delta_y<0:
                if on_left:
                    starting_grid[1] = starting_grid[1] - 1
                if on_bottom:
                    starting_grid[0] = starting_grid[0] - 1 
            elif delta_x <0:
                if on_left:
                    starting_grid[1] = starting_grid[1] - 1

            elif delta_y<0:
                if on_bottom:
                    starting_grid[0] = starting_grid[0] - 1 

        
        last_added_grid = starting_grid
        grids_to_check = [[last_added_grid[0],last_added_grid[1]]]
        
        if gradient<1: #path is horizontally dominant (the variable gradient is actually the absolute value of gradient and so is never negative)
            #print('path is horizontally dominant')

            
            if on_left and on_bottom: #starting node is at corner of an og square
                d = gradient

            elif on_left: #starting node is on the vertical line between two og squares
                origin_node = self.og_matrix_to_map_conversion(starting_grid,og_resolution=og_resolution)
                d = gradient + abs(0.5 - (delta_y/(2*abs(delta_y))) - (start.y-origin_node.y)*ratio)
                
                

            elif on_bottom: #starting node is on the vertical line between two og squares
                origin_node = self.og_matrix_to_map_conversion(starting_grid,og_resolution=og_resolution)
                d = gradient * abs(0.5 + (delta_x/(2*abs(delta_x))) - (start.x - origin_node.x) * ratio)

                
            
            else: #most likely case, if not the only possible one
                origin_node = self.og_matrix_to_map_conversion(starting_grid,og_resolution=og_resolution)
                d = abs(0.5 - (delta_y/(2*abs(delta_y))) - (start.y-origin_node.y)*ratio) + gradient * abs(0.5 + (delta_x/(2*abs(delta_x))) - (start.x - origin_node.x) * ratio)
                
        
            
            #print('tracing the path')
            #print()
            #print('starting grid is: (',starting_grid[0],',',starting_grid[1],')')  
            while last_added_grid != end_grid:
                counter=counter+1
                #if counter >20:
                    #print('loop terminating')
                    #break
                if d >=1:
                    d = d-1
                    last_added_grid[0] = int(last_added_grid[0]+delta_y/abs(delta_y))
                    grids_to_check.append([last_added_grid[0],last_added_grid[1]])
                    continue
                last_added_grid[1] = int(last_added_grid[1]+delta_x/abs(delta_x))

                #print('adding a grid in the HORIZONTAL DOMINANT direction: (',last_added_grid[0],',',last_added_grid[1],')') 
                
                grids_to_check.append([last_added_grid[0],last_added_grid[1]])
                d = d + gradient
            #print('FINAL GRID IS: (',last_added_grid[0],',',last_added_grid[1],')')     
            #print('END GRID IS: (',end_grid[0],',',end_grid[1],')')   

            
            


        else: #path is vertically dominant
            #print('path is vertically dominant')
            gradient = 1/gradient

            if on_left and on_bottom: #starting node is at corner of an og square
                d = gradient

            elif on_left: #starting node is on the vertical line between two og squares
                origin_node = self.og_matrix_to_map_conversion(starting_grid,og_resolution=og_resolution)
                d = gradient * abs(0.5 + (delta_y/(2*abs(delta_y))) - (start.y-origin_node.y)*ratio)
                
                

            elif on_bottom: #starting node is on the vertical line between two og squares
                origin_node = self.og_matrix_to_map_conversion(starting_grid,og_resolution=og_resolution)
                d = gradient + abs(0.5 - (delta_x/(2*abs(delta_x))) - (start.x - origin_node.x) * ratio)

                
            
            else: #most likely case, if not the only possible one
                origin_node = self.og_matrix_to_map_conversion(starting_grid,og_resolution=og_resolution)
                d = gradient * abs(0.5 + (delta_y/(2*abs(delta_y))) - (start.y-origin_node.y)*ratio) + abs(0.5 - (delta_x/(2*abs(delta_x))) - (start.x - origin_node.x) * ratio)


            #print('tarcing the path')
            #print()
            #print('starting grid is: (',starting_grid[0],',',starting_grid[1],')')      
            while last_added_grid != end_grid:
                counter=counter+1
                #if counter >20:
                    #print('loop terminating')
                    #break
                if d >=1:
                    d = d-1
                    last_added_grid[1] = int(last_added_grid[1]+delta_x/abs(delta_x))
                    grids_to_check.append([last_added_grid[0],last_added_grid[1]]) 
                    continue
                last_added_grid[0] = int(last_added_grid[0]+delta_y/abs(delta_y))
                #print('adding a grid in the VERTICAL DOMINANT direction: (',last_added_grid[0],',',last_added_grid[1],')') 
                
                grids_to_check.append([last_added_grid[0],last_added_grid[1]])
                d = d + gradient
            #print('FINAL GRID IS: (',last_added_grid[0],',',last_added_grid[1],')')
            #print('END GRID IS: (',end_grid[0],',',end_grid[1],')')   
        

        #print()
        #print("grids to check are:")
        #for grid in grids_to_check:
            #print("(",grid[0],',',grid[1],')') 


        #print("cheking the traced path:")
        for grid in grids_to_check:
            if og_data[grid[0]][grid][1] !=0:
                return False
        return True
       
            

               
    def generate_final_course(self, start_mid_point, end_mid_point):
        """
        Reconstruct path from start to end node
        """
        # First half
        node = self.start_node_list[-1]
        path = []
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
            '''if node.parent is None:
                print("Node:","(",round(node.x,3),",",round(node.y,3),");","Has no parent")'''
        path.append([node.x, node.y])
        #print("final path is:")

        '''for node in path:
            print("(",round(node[0],3),",",round(node[1],3),")")'''
        # Other half
        """
        node = self.end_node_list[end_mid_point]
        path = path[::-1]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        """
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        
        #x = 10*self.expand_dis*np.random.random_sample()
        #y = 10*self.expand_dis*np.random.random_sample()
        x = self.map_width * np.random.random_sample()
        y = self.map_height * np.random.random_sample()
        rnd = self.Node(x, y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):        
        # Compute Euclidean disteance between rnd_node and all nodes in tree
        # Return index of closest element
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)** 2 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta     









#Modify generate path func
# return a path array in the rrt function