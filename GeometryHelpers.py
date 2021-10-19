import numpy as np
import math

import itertools

CHECKPOINT_DISTANCE_THRESH = 0.1 # cm

def pointgroup_to_cone(group): # Numpy could help speed this averaging operation
    average_x = 0
    average_y = 0
    for point in group:
        average_x += point['x']
        average_y += point['y']
    average_x = average_x / len(group)
    average_y = average_y / len(group)
    return {'x': average_x, 'y': average_y}

def distance(x1, y1, x2, y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5 

def circle_intersects_with_line(circle_x, circle_y, circle_r, p1_x, p1_y, p2_x, p2_y): 
    """
        Drop a perpendicular from C((circle_x, circle_y), circle_r) to line L((p1_x, p1_y), (p2_x, p2_y)) as (perp_x, perp_y)
        return True if
            1. If the point on the line is between (p1_x, p1_y), (p2_x, p2_y)
            2. distance( circle_x, circle_y,  perp_x, perp_y) < circle_r
        return False otherwise
    """
    try:
        m = (p2_y-p1_y)/(p2_x-p1_x)
    except ZeroDivisionError:
        m = 5729577.951308174 # tan(89.99999)

    if m == 0:
        m = 0.000000175 # cot(89.99999)

    # Perpenicular line
    A1 = 1.0/m
    B1 = 1.0
    C1 = -(circle_y + (1.0/m)*circle_x)

    # Original line
    A2 = m
    B2 = -1.0
    C2 = -m * p1_x + p1_y  

    side_of_line = A2*circle_x + B2*circle_y + C2

    tmp = A1*B2 - A2*B1 # tmp can never be zero as we have defined the two lines to be perpendicular
    
    perp_x = (B1*C2 - B2*C1) / tmp
    perp_y = (A2*C1 - A1*C2) / tmp

    d = distance(perp_x, perp_y, circle_x, circle_y)

    #if d < circle_r and (perp_y - p1_y)/(perp_x - p1_x) * (p2_y - perp_y)/(p1_x - perp_x) >= 0:
    if d < circle_r and ((perp_y - p1_y)*(p2_y - perp_y) >= 0 and (perp_x - p1_x)*(p1_x - perp_x) >=0):
        return True, d, (perp_x, perp_y), side_of_line

    return False, d, (perp_x, perp_y), side_of_line

"""
Approach 1 : 6 nearest cones : 

When considering the 3 nearby checkpoints, we get 6 cones
The 3 cones on left boundary are :
nearby_checkpoints[0]['c1'], nearby_checkpoints[1]['c1'], nearby_checkpoints[3]['c1'] # Why 0,1,3? why not 0,1,2?

The 3 cones on right boundary is :
nearby_checkpoints[0]['c2'], nearby_checkpoints[1]['c2'], nearby_checkpoints[3]['c2']
            
Where nearby_checkpoints[i] is 
{
    'x': (c2['x'] + c1['x'])/2,     # Midpoint of the 2 cones
    'y': (c2['y'] + c1['y'])/2,     # Midpoint of the 2 cones
    'c1': c1,                       # Left cone
    'c2': c2,                       # Right cone
    'visited': False                # Flag to see if cp is already visited 
}

These 6 cones form a polygon.
The car is inside the track iff:
    1. point_is_inside_polygon(car_x, car_y, [
        nearby_checkpoints[0]['c1'], nearby_checkpoints[1]['c1'], nearby_checkpoints[3]['c1'],
        nearby_checkpoints[0]['c2'], nearby_checkpoints[1]['c2'], nearby_checkpoints[3]['c2']
    ])

Approach 2 : Full Track Analysis : 

The track is two polygons :
    1. p_in  - inner set of cones
    2. p_out - outer set of cones

The car is inside the track iff :
    1. point_is_inside_polygon(car_x, car_y, p_in) == False
    2. point_is_inside_polygon(car_x, car_y, p_out) == True
"""
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
def point_is_inside_polygon(p_x, p_y, polygon_points):
    """
        How to check if a given point lies inside or outside a polygon?
            https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/#:~:text=1)%20Draw%20a%20horizontal%20line,on%20an%20edge%20of%20polygon.

        The order set of polygon_points must be considered such that :
            1. We start with a point p
            2. The next point p' is the closest point to p ; mark p as visited
            3. The third point p'' is the point closest to p', but not marked visited list 
        
        We repeat until all points are marked as visited
        The last line will be between the first point marked as visited and the last point

        polygon_points[i] = {
            'x': float,
            'y': float,
        }

        https://stackoverflow.com/questions/36399381/whats-the-fastest-way-of-checking-if-a-point-is-inside-a-polygon-in-python
    """
    polygon_points = list(map(lambda p: Point(p['x'], p['y']) , polygon_points))

    point = Point(p_x, p_y)
    polygon = Polygon(polygon_points)
    if polygon.is_valid:
        return polygon.contains(point)
    else:
        return "INVALID"


def point_is_inside_polygon_buggy(p_x, p_y, polygon_points_tot):
    """
        How to check if a given point lies inside or outside a polygon?
            https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/#:~:text=1)%20Draw%20a%20horizontal%20line,on%20an%20edge%20of%20polygon.

        The order set of polygon_points must be considered such that :
            1. We start with a point p
            2. The next point p' is the closest point to p ; mark p as visited
            3. The third point p'' is the point closest to p', but not marked visited list 
        
        We repeat until all points are marked as visited
        The last line will be between the first point marked as visited and the last point

        polygon_points[i] = {
            'x': float,
            'y': float,
        }

        https://stackoverflow.com/questions/36399381/whats-the-fastest-way-of-checking-if-a-point-is-inside-a-polygon-in-python
    """

    
    for polygon_points in itertools.permutations( polygon_points_tot ): # all_combintations
        #print(polygon_points)
        polygon_points = list(map(lambda p: Point(p['x'], p['y']) , polygon_points))

        """
        polygon_points_sorted = []
        while len(polygon_points)>1:
            closest_point, d, index = polygon_points[0], distance(polygon_points[0][0], polygon_points[0][1], p_x, p_y), 0
            for i in range(len(polygon)):
                pass
        """

        point = Point(p_x, p_y)
        polygon = Polygon(polygon_points)
        if polygon.is_valid:
            #print("=============polygon.is_valid=================")
            res = polygon.contains(point)
            #print(polygon)
            #print((p_x, p_y))
            if res:
                return True
    
    return False