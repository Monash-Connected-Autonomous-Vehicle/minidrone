import math


def naive_bresenham(p1, p2):
    """
    Takes 2 points with integer coordinates and outputs a list of integer coordinates approximating the line between
    them using a naive bresenham algorithm involving floating point arithmetic. Utilises the cartesian coord system.

    :param p1: integer coordinate as a tuple (x, y)
    :param p2: integer coordinate as a tuple (x, y)
    :return: list of integer coordinates as tuples [(x1, y1), ..., (xn, yn)]
    """
    # determine the left most point and the right most point
    if p1[0] <= p2[0]:
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
    else:
        x1, y1 = p2[0], p2[1]
        x2, y2 = p1[0], p1[1]

    # determine the gradient and y-int
    dx = x2 - x1
    dy = y2 - y1
    try:
        m = dy/dx
    except ZeroDivisionError:
        return [(x1, y) for y in range(min(y1, y2), max(y1, y2) + 1)]
    c = y1 - (m * x1)

    # determine the always incrementing axis and the slow sometimes-incrementing axis
    # find all coordinates for all integer values in the domain of the always incrementing axis
    if 0 <= abs(m) <= 1:  # m is from -1 to 1 excluding 0
        increment = range(math.floor(x1), math.floor(x2) + 1)
        y = [round(m*x + c) for x in increment]  # consider that x.5 rounds up, but should it?
        x = [x for x in increment]
    elif 1 < abs(m):  # m is from +/-1 to +/-inf
        increment = range(math.floor(min(y1, y2)), math.floor(max(y1, y2)) + 1)
        x = [round((y-c)/m) for y in increment]
        y = [y for y in increment]

    return [(x[i], y[i]) for i in range(len(x))]
