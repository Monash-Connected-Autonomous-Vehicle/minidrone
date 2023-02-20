import numpy as np

import math
from math import pi, abs

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
