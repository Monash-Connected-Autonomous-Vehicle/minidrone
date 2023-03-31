#!/usr/bin/env python3

from matplotlib import pyplot as plt
from math import pi

import numpy as np
import math
from typing import Callable

from scipy.signal import convolve2d
from scipy.ndimage import label

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
    ax[1,1].imshow(eroded[:, :, 0])
    ax[2,1].imshow(eroded[:, :, 1])
    ax[0,1].imshow(space.segment())
    plt.show()
