import numpy as np

from math import sin, cos, pi, abs

def sample_trajectory(angle: float, grid_width: float, min_step: float, step_dist: float):
    """
    Sample equidistant points along a circular trajectory tangent to x axis.
    Trajectory intersects inscribed circle of square grid at specified point.

    Paramters
    ---------
    angle: Angle between x axis and line segment from origin to point of inscribed circle intersection
    grid_width: side length of square gird
    min_step: minimum distance along trajectory for points to be sampled from
    step_dist: distance between subsequent sampled points
    """

    if angle == 0:  # Special case for straight trajectory
        x = np.arange(min_step, grid_width/2, step_dist)
        return np.stack(x, np.zeros(len(x))).T

    else:  # Sample points along circular trajectory
        sample_points = []
        r = grid_width/(2*sin(angle))

        step = min_step
        ang = step/r
        x, y = r*sin(ang), r*(1-cos(ang))

        while abs(x) < grid_width/2 and abs(y) < grid_width/2 and abs(ang) < pi/2:
            sample_points.append((x, y))
            step += step_dist
            ang = step/r
            y = r*sin(ang), r*(1-cos(ang))
        
        return np.array(sample_points)
