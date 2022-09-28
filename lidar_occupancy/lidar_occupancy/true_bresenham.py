def true_bresenham(p1, p2):
    """
    Takes 2 points with integer coordinates and outputs a list of integer coordinates approximating the line between
    them using bresenham's algorithm involving integer only arithmetic.

    :param p1: integer coordinate as a tuple (x, y)
    :param p2: integer coordinate as a tuple (x, y)
    :return: list of integer coordinates as tuples [(x1, y1), ..., (xn, yn)]
    """
    # determine the left most point and the right most point
    if p1[0] <= p2[0]:
        x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
    else:
        x1, y1, x2, y2 = p2[0], p2[1], p1[0], p1[1]

    # rise and run
    dx, dy = x2 - x1, y2 - y1

    # if dy > dx, recursive call on reflection on y=x, undo reflection, then return
    if abs(dy) > abs(dx):
        points = true_bresenham((y1, x1), (y2, x2))
        points = [(p[1], p[0]) for p in points]
        return points

    # if negative gradient, recursive call on reflection on y=0, undo reflection, then return
    if dy < 0 or dx < 0:
        points = true_bresenham((x1, -y1), (x2, -y2))
        points = [(p[0], -p[1]) for p in points]
        return points

    # start with (x1, y1) as first point
    xk, yk = x1, y1
    points = [p1]

    # decision variable for next point
    # pk > 0 then line is closer to top, yk + 1
    # pk < 0 then line is closer to bottom, yk
    PK_CONST = dx * (2*y1 - 1) + 2*dy * (1 - x1)
    pk = 2 * (dy*xk - dx*yk) + PK_CONST

    while xk < x2:
        # determine next point
        if pk >= 0:
            yk += 1
        xk += 1
        points.append((xk, yk))

        # update decision variable
        pk = 2 * (dy*xk - dx*yk) + PK_CONST

    return points
