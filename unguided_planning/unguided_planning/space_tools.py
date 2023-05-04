from matplotlib import pyplot as plt
from math import pi

import numpy as np
import math
from typing import Callable

from scipy.signal import convolve2d
from scipy.ndimage import label
from scipy.optimize import minimize

class OccSpace:
    def __init__(self, grid) -> None:
        '''
        Initialize OccSpace object based on occupancy grid

        Parameters
        ----------
        grid: occupancy grid numpy array, where -1 is lane, 0 is unknown, 1 is driveable
        '''
        self.grid = grid
    
    def generate_mesh_grid(self, resolution) -> None:
        # Sample mesh grid with number of points horizontally equal to resolution
        # Array of edge statuses for each edge, indexed by vertex coordinates and vertical/horizontal
        self.sample_res = resolution
        self.edge_array = np.zeros((resolution, resolution, 2), dtype=bool)
        sample_x = np.linspace(0, self.grid.shape[0]-1e-5, resolution)  # Vertex coordinates of sampled mesh
        sample_y = np.linspace(0, self.grid.shape[1]-1e-5, resolution)
        
        # Iterate through sampled vertices, and record the occupancy of each edge
        for i in range(resolution):
            for j in range(resolution):
                if i < resolution-1:
                    # Record occupancy of horizontal edge
                    xx, y = sample_x[i:i+2].astype(int), sample_y[j].astype(int)  # x span, y coordinate (in raw grid coords)
                    self.edge_array[i, j, 1] = np.all(self.grid[xx, y] == 1) and not \
                                               np.any(self.grid[xx[0]:xx[1], j] == -1)

                if j < resolution-1:
                    # Record occupancy of vertical edge
                    x, yy = sample_y[i].astype(int), sample_y[j:j+2].astype(int)  # x coordinate, y span (in raw grid coords)
                    self.edge_array[i, j, 0] = np.all(self.grid[x, yy] == 1) and not \
                                               np.any(self.grid[i, yy[0]:yy[1]] == -1)
        return self.edge_array

    def erode_mesh(self):
        '''
        removes edges that do not have any adjacent parallel neighbours
        '''
        kernel = np.ones((1,3), dtype=int)
        for i, k in enumerate([kernel, kernel.T]):
            self.edge_array[:, :, i] *= convolve2d(self.edge_array[:, :, i], k, 
                                                   mode='same', boundary='fill', 
                                                   fillvalue=1) > 1
        return self.edge_array
    
    def segment(self):
        '''Returns a connected components labeling of drivable edge occupied areas'''
        self.mesh_areas = np.sum((self.edge_array[:-1, :-1, 0], self.edge_array[:-1, :-1, 1],
                                  self.edge_array[1:, 1:, 0], self.edge_array[1:, 1:, 1]), axis=0) > 1
        return label(self.mesh_areas)[0]
    
    def trajectory_squares(self, parametric: Callable, inverse_x: Callable, inverse_y: Callable):
        '''
        Determine all sample grid squares that a parametric tajectory, defined with coordinates in mesh_areas
        '''
        t = 0
        x, y = parametric(t)
        intersected = [(int(x), int(y))]
        # iterate through pixels until grid is exited
        while x < self.sample_res and y < self.sample_res:
            
            x, y = parametric(t)

            
            


class AngularTrajectory:
    def __init__(self, angle: float, grid_width: float) -> None:
        """
        Circular trajectory tangent to x axis intersecting inscribed circle of square grid at specified point.

        Parameters
        ----------
        angle: Angle between x axis and line segment from origin to point of inscribed circle intersection
        grid_width: side length of square gird
        """
        self.angle = angle
        self.grid_width = grid_width

        if angle == 0:
            self.r = math.inf
            self.parametric = lambda t: (t, 0)
            self.cartesian = None

        else:
            self.r = grid_width/(2*math.sin(angle))
            self.parametric = lambda t: (self.r*math.sin(t/self.r), self.r*(1-math.cos(t/self.r)))
            self.cartesian = lambda x: math.sqrt(x*(2*self.r-x))

    def sample(self, min_step: float, step_dist: float):
        """
        Sample equidistant points along trajectory

        Parameters
        ---------
        min_step: minimum distance along trajectory for points to be sampled from
        step_dist: distance between subsequent sampled points
        """

        if self.angle == 0:  # Special case for straight trajectory
            x = np.arange(min_step, self.grid_width/2, step_dist)
            return np.stack(x, np.zeros(len(x))).T

        else:  # Sample points along circular trajectory
            sample_points = []
            step = min_step
            x, y = self.parametric(step)

            while abs(x) < self.grid_width/2 and abs(y) < self.grid_width/2 and abs(step/self.r) < pi/2:
                sample_points.append((x, y))
                step += step_dist
                x, y = self.parametric(step)
            
            return np.array(sample_points)
    
class FindAngle:
    def __init__(self, grid):
        self.grid = grid
        self.grid_width = grid.shape[0]
        self.half_grid = int(self.grid_width/2)
        
    def find_angle(self):

        a0 = [0]
        bounds = [(-90, 90)]

        return minimize(self.check, a0, method = 'SLSQP', bounds = bounds)
        """
        if r >= l:
            m = (r-l)//2

            Done, dir = check(self.ang_range[m])

            if Done:
                return m
            
            elif dir:  
                return find_angle(m+1,r)
            else:
                return find_angle(l,m-1)
        else:
            return "No root found"
    """
    def check(self,angle: float):
        #Creating trajectory from the angle 
        traj = AngularTrajectory(angle,self.grid_width)
        
        #Case of a 0 angle
        if angle == 0:
            left = np.sum(self.grid[:,0:self.half_grid]==2)
            right = np.sum(self.grid[:,self.half_grid:self.grid_width-1]==2)

            return abs(left - right)
            """
            if left > right:
                return False, False
            elif right > left:
                return False, True 
            else:
                return True, True
"""
        #Find the radius
        r = int(traj.r)

        #Grid for storing if it is in the arc
        circle = np.zeros((self.grid.shape))

        #setting the initial conditions 
        x_i = 0
        y_i = r
        d_par = 1-r
        e = r-self.half_grid #value to check the error of the circle drawing

        while (x_i < y_i) & ((x_i < self.grid_width)|(y_i < self.grid_width)):
            #fill the in_circle grid with ones to show what is inside 
            print(x_i)
            print(y_i)
 
            if angle < 0:
                if x_i >= e:
                    circle[y_i:,x_i-e], circle[self.grid_width-1-x_i,0:y_i-e] = 1,1
                elif y_i <= self.grid_width -1:
                    circle[self.grid_width-1-x_i,0:y_i-e] = 1
            elif angle > 0:
                if x_i >= e:
                    circle[y_i:,self.grid_width -1 - (x_i-e)], circle[self.grid_width-1-x_i,self.half_grid - (y_i-e):] = 1,1
                elif x_i <= self.grid_width -1:
                    circle[self.grid_width -1 -x_i,self.half_grid - (y_i-e):] = 1
            
            #update the decision parameter and x_i and y_i
            if d_par < 0:
                d_par = d_par + 2*x_i + 3
                x_i = x_i + 1
            if d_par >= 0:
                d_par = d_par + 2*(x_i-y_i) + 5
                x_i = x_i + 1
                y_i = y_i - 1
        
        #Checking how much lies within and how much lies out
        in_circle = np.sum((circle == 1) & (self.grid==2))
        out_circle = np.sum((circle == 0) & (self.grid==2))

        return abs(in_circle - out_circle)
        """
        if in_circle > out_cicle:
            if angle < 0:
                return False, False
            else:
                return False, True
        elif out_circle > in_circle:
            if angle < 0:
                return False, True
            else: 
                return False, False
        else:
            return True, True
"""



if __name__ == '__main__':
    grid = np.ones((100, 100))
    grid[:, 20:30] = -1
    grid[:, 70:80] = -1
    grid[:, 30:70] = 1
    grid[60:70, 20:30] = 1
    grid[15:25, 70:80] = 1
    grid[30:60, 40:60] = 0
    
    space = OccSpace(grid)
    edges = space.generate_mesh_grid(20)

    fig, ax = plt.subplots(3, 2)
    ax[0,0].imshow(space.segment())
    ax[1,0].imshow(edges[:, :, 0])
    ax[2,0].imshow(edges[:, :, 1])

    eroded = space.erode_mesh()
    seg = space.segment()
    ax[1,1].imshow(eroded[:, :, 0])
    ax[2,1].imshow(eroded[:, :, 1])
    ax[0,1].imshow(space.segment())

    angle = FindAngle(space.segment())

    a = angle.find_angle()
    print(a)
    
    
    