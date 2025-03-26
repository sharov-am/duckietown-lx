import math
import itertools
import random
from typing import List
from dataclasses import dataclass

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

__all__ = ["CollisionChecker"]


@dataclass
class RotatedRectangle:
    x1: float
    y1: float
    x2: float
    y2: float
    x3: float
    y3: float
    x4: float
    y4: float

    def get_line_segments(self)-> List:
        yield  (self.x1,self.y1), (self.x2,self.y2)
        yield  (self.x2,self.y2), (self.x3,self.y3)
        yield  (self.x3,self.y3), (self.x4,self.y4)
        yield  (self.x4,self.y4), (self.x1,self.y1)

    def get_vertices(self)-> List:
        yield  (self.x1,self.y1)
        yield  (self.x2,self.y2)
        yield  (self.x3,self.y3)
        yield  (self.x4,self.y4)

    def get_x_min_vertice(self)-> float:
        return min([self.x1, self.x2, self.x3, self.x4])

    def get_x_max_vertice(self)-> float:
        return max([self.x1, self.x2, self.x3, self.x4])

    def get_y_min_vertice(self)-> float:
        return min([self.y1, self.y2, self.y3, self.y4])

    def get_y_max_vertice(self)-> float:
        return max([self.y1, self.y2, self.y3, self.y4])

    def get_y_max_vertice(self)-> float:    
        return max([self.y1, self.y2, self.y3, self.y4])

    def get_lowest_side_vertices(self):
        """
        Get the lowest side of a rectangle given its four vertices.

        Parameters:
            vertices (list): List of (x, y) tuples representing the rectangle's vertices.

        Returns:
            list: The two vertices defining the lowest side of the rectangle.
        """
        # Sort vertices by y-coordinate (ascending order)
        vertices = [(self.x1,self.y1), (self.x2,self.y2), (self.x3,self.y3), (self.x4,self.y4)]
        sorted_vertices = sorted(vertices, key=lambda v: v[1])

        # The two vertices with the smallest y-coordinates define the lowest side
        lowest_side = sorted_vertices[:2]

        return lowest_side[0],lowest_side[1]


    def get_lowest_side_vertices(self):
        """
        Get the lowest side of a rectangle given its four vertices.

        Parameters:
            vertices (list): List of (x, y) tuples representing the rectangle's vertices.

        Returns:
            list: The two vertices defining the lowest side of the rectangle.
        """
        # Sort vertices by y-coordinate (ascending order)
        vertices = [(self.x1,self.y1), (self.x2,self.y2), (self.x3,self.y3), (self.x4,self.y4)]
        sorted_vertices = sorted(vertices, key=lambda v: v[1])

        # The two vertices with the smallest y-coordinates define the lowest side
        lowest_side = sorted_vertices[:2]

        return lowest_side[0],lowest_side[1]

    def get_highest_vertices(self):
        
        # Sort vertices by y-coordinate (ascending order)
        vertices = [(self.x1,self.y1), (self.x2,self.y2), (self.x3,self.y3), (self.x4,self.y4)]
        sorted_vertices = sorted(vertices, key=lambda v: v[1],reverse = True)

        # The two vertices with the smallest y-coordinates define the lowest side
        lowest_side = sorted_vertices[:2]

        return highest_side[1],lowest_side[0]

    def get_leftmost_vertices(self):
        
        # Sort vertices by y-coordinate (ascending order)
        vertices = [(self.x1,self.y1), (self.x2,self.y2), (self.x3,self.y3), (self.x4,self.y4)]
        sorted_vertices = sorted(vertices, key=lambda v: v[0])

        # The two vertices with the smallest y-coordinates define the lowest side
        lowest_side = sorted_vertices[:2]

        return highest_side[0],lowest_side[1]

    def get_rightmost_vertices(self):    
        
        # Sort vertices by y-coordinate (ascending order)
        vertices = [(self.x1,self.y1), (self.x2,self.y2), (self.x3,self.y3), (self.x4,self.y4)]
        sorted_vertices = sorted(vertices, key=lambda v: v[0], reverse = True)

        # The two vertices with the smallest y-coordinates define the lowest side
        lowest_side = sorted_vertices[:2]

        return highest_side[1],lowest_side[0]



class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
                
        context.info("init()")
        

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            environment=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(
    environment: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    # TODO you can start by rototranslating the robot_body by the robot_pose
    rototranslated_robot: List[PlacedPrimitive] = [rototranslate_robot(robot_pose=robot_pose, robot_body=r_b) for r_b in robot_body]

    # Then, call check_collision_list to see if the robot collides with the environment
    collided = check_collision_list(rototranslated_robot, environment)

    # TODO return the status of the collision
    # for now let's return a random guess
    return collided



def check_collision_list(
    rototranslated_robot: List[PlacedPrimitive], environment: List[PlacedPrimitive]
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    for robot, envObject in itertools.product(rototranslated_robot, environment):
        if check_collision_shape(robot, envObject):
            return True

    return False


def circle_intersection(a: PlacedPrimitive, b: PlacedPrimitive)->bool:
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        center_distance = ((a.pose.x - b.pose.x)**2 + (a.pose.y - b.pose.y)**2 )**.5
        radius_sum = a.primitive.radius + b.primitive.radius
        return center_distance <= radius_sum
    raise Exception("not circles used for circle_intersection")




def rotate_point(x, y, center_x, center_y, angle_degrees):
    """
    Rotate a point (x, y) around a center point (center_x, center_y) by a given angle in degrees.

    Parameters:
        x (float): X-coordinate of the point to rotate.
        y (float): Y-coordinate of the point to rotate.
        center_x (float): X-coordinate of the rotation center.
        center_y (float): Y-coordinate of the rotation center.
        angle_degrees (float): Angle of rotation in degrees.

    Returns:
        tuple: The new (x, y) coordinates after rotation.
    """
    # Convert angle from degrees to radians
    angle_radians = math.radians(angle_degrees)
    
    # Translate point to origin (relative to the center)
    translated_x = x - center_x
    translated_y = y - center_y
    
    # Apply rotation transformation
    new_x = translated_x * math.cos(angle_radians) - translated_y * math.sin(angle_radians)
    new_y = translated_x * math.sin(angle_radians) + translated_y * math.cos(angle_radians)
    
    # Translate back to the original coordinate system
    new_x += center_x
    new_y += center_y
    
    return (new_x, new_y)


def rotate_rectangle3(x1, y1, x2, y2,c_x1,c_x2,c_y1,c_y2, angle_degrees)-> RotatedRectangle:
    """
    Rotate a rectangle defined by bottom-left (x1, y1) and upper-right (x2, y2) coordinates.

    Parameters:
        x1 (float): Bottom-left x-coordinate.
        y1 (float): Bottom-left y-coordinate.
        x2 (float): Upper-right x-coordinate.
        y2 (float): Upper-right y-coordinate.
        angle_degrees (float): Angle of rotation in degrees.

    Returns:
        tuple: New bottom-left and upper-right coordinates of the rotated rectangle.
    """
    # Calculate the center of the rectangle
    center_x = (c_x1*x1 + c_x2*x2) 
    center_y = (c_y1*y1 + c_y2*y2) 
    
    # Define all four vertices of the rectangle
    vertices = [
        (x1, y1),  # Bottom-left
        (x2, y1),  # Bottom-right
        (x2, y2),  # Upper-right
        (x1, y2)   # Upper-left
    ]
    
    # Rotate all vertices
    rotated_vertices = [rotate_point(x, y, center_x, center_y, angle_degrees) for x, y in vertices]
    
    return RotatedRectangle(rotated_vertices[0][0], rotated_vertices[0][1], rotated_vertices[1][0], rotated_vertices[1][1],
    rotated_vertices[2][0], rotated_vertices[2][1], rotated_vertices[3][0], rotated_vertices[3][1])



def rotate_rectangle2(x1, y1, x2, y2, angle_degrees)-> RotatedRectangle:
    """
    Rotate a rectangle defined by bottom-left (x1, y1) and upper-right (x2, y2) coordinates.

    Parameters:
        x1 (float): Bottom-left x-coordinate.
        y1 (float): Bottom-left y-coordinate.
        x2 (float): Upper-right x-coordinate.
        y2 (float): Upper-right y-coordinate.
        angle_degrees (float): Angle of rotation in degrees.

    Returns:
        tuple: New bottom-left and upper-right coordinates of the rotated rectangle.
    """
    # Calculate the center of the rectangle
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    
    # Define all four vertices of the rectangle
    vertices = [
        (x1, y1),  # Bottom-left
        (x2, y1),  # Bottom-right
        (x2, y2),  # Upper-right
        (x1, y2)   # Upper-left
    ]
    
    # Rotate all vertices
    rotated_vertices = [rotate_point(x, y, center_x, center_y, angle_degrees) for x, y in vertices]
    
    return RotatedRectangle(rotated_vertices[0][0], rotated_vertices[0][1], rotated_vertices[1][0], rotated_vertices[1][1],
    rotated_vertices[2][0], rotated_vertices[2][1], rotated_vertices[3][0], rotated_vertices[3][1])



def rototranslate_robot(robot_pose:FriendlyPose, robot_body: PlacedPrimitive )-> PlacedPrimitive:
   
    new_theta_deg = (robot_body.pose.theta_deg + robot_pose.theta_deg) % 360

    total_pose_x_shift = robot_pose.x + robot_body.pose.x 
    total_pose_y_shift = robot_pose.y + robot_body.pose.y 
       
    return PlacedPrimitive(FriendlyPose(total_pose_x_shift, total_pose_y_shift, new_theta_deg ),robot_body.primitive)


def get_primitive_bounded_rectangle2(any_rectangular_body: PlacedPrimitive) -> RotatedRectangle:
    
    x1 = any_rectangular_body.pose.x + any_rectangular_body.primitive.xmin 
    x2 = any_rectangular_body.pose.x + any_rectangular_body.primitive.xmax
    y1 = any_rectangular_body.pose.y + any_rectangular_body.primitive.ymin 
    y2 = any_rectangular_body.pose.y + any_rectangular_body.primitive.ymax
        
    rotate_angle = any_rectangular_body.pose.theta_deg
    
    c_x1 = abs(any_rectangular_body.primitive.xmax)/(abs(any_rectangular_body.primitive.xmax) + abs(any_rectangular_body.primitive.xmin))
    c_x2 = abs(any_rectangular_body.primitive.xmin)/(abs(any_rectangular_body.primitive.xmax) + abs(any_rectangular_body.primitive.xmin))
    c_y1 = abs(any_rectangular_body.primitive.ymax)/(abs(any_rectangular_body.primitive.ymax) + abs(any_rectangular_body.primitive.ymin))
    c_y2 = abs(any_rectangular_body.primitive.ymin)/(abs(any_rectangular_body.primitive.ymax) + abs(any_rectangular_body.primitive.ymin)) 


    return rotate_rectangle3(x1, y1, x2, y2, c_x1,c_x2, c_y1, c_y2 ,rotate_angle + 0.01) 



def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    
    # TODO check if the two primitives are colliding
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
       print("circle and circle")
       return circle_intersection(a,b)

    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        #print(f"a = {a}, b = {b}")
                
        #print(f"rectangle and circle intersection:{result}")
        
        robot_rotated_rectangle = get_primitive_bounded_rectangle2(a)
        result = is_rectangle_and_circle_intersect2( robot_rotated_rectangle,  b.pose.x, b.pose.y, b.primitive.radius)
        return result
    
    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        
        robot_rotated_rectangle = get_primitive_bounded_rectangle2(a)
        obstacle_rotated_rectangle = get_primitive_bounded_rectangle2(b)

        result = rectangle_intersection2(obstacle_rotated_rectangle, robot_rotated_rectangle) or \
                 rectangle_intersection2(robot_rotated_rectangle,obstacle_rotated_rectangle) 
        #print(f"rectangle and rectangle intersection:{result}")
        return result
 
    return False



def perpendicular_from_point(x1, y1, x2, y2, x, y):
    """
    Calculate the perpendicular from a point (x, y) to a line defined by two points (x1, y1) and (x2, y2).

    Parameters:
        x1 (float): X-coordinate of the first point on the line.
        y1 (float): Y-coordinate of the first point on the line.
        x2 (float): X-coordinate of the second point on the line.
        y2 (float): Y-coordinate of the second point on the line.
        x (float): X-coordinate of the point.
        y (float): Y-coordinate of the point.

    Returns:
        tuple: Coordinates of the foot of the perpendicular (intersection point).
    """
    # Calculate the slope of the original line
    if x2 != x1:
        m = (y2 - y1) / (x2 - x1)
    else:
        # Line is vertical, perpendicular is horizontal
        return (x1, y)

    # Calculate the slope of the perpendicular line
    if m == 0:
        # Original line is horizontal, perpendicular is vertical
        return (x, y1)
    else:
        m_perp = -1 / m

    # Equation of the original line: y - y1 = m(x - x1)
    # Equation of the perpendicular line: y - y = m_perp(x - x)

    # Solve for the intersection point
    # Original line: y = m(x - x1) + y1
    # Perpendicular line: y = m_perp(x - x) + y
 
    # Set the two equations equal to each other:
    # m(x - x1) + y1 = m_perp(x - x) + y

    # Solve for x:
    x_intersect = (m * x1 - m_perp * x + y - y1) / (m - m_perp)

    # Substitute x_intersect into one of the equations to find y_intersect
    y_intersect = m * (x_intersect - x1) + y1

    return (x_intersect, y_intersect)

def is_rectangle_and_circle_intersect2(rotated_rectangle:RotatedRectangle,x:float,y:float,radius:float) -> bool:
   
    for rect_x, rect_y in rotated_rectangle.get_vertices():
        if ((rect_x - x)**2 + (rect_y - y)**2) < radius**2:
            #print("CASE1")
            return True
    
    
    for segment1,segment2 in rotated_rectangle.get_line_segments():
              
       (x_intersect, y_intersect) = perpendicular_from_point(segment1[0],segment1[1],segment2[0],segment2[1],x,y)
       
       dist_from_center = ((x - x_intersect)**2 + (y - y_intersect)**2)**.5
       
       if dist_from_center <= radius and min(segment1[0],segment2[0]) <= x_intersect and x_intersect <= max(segment1[0],segment2[0]) and \
          min(segment1[1], segment2[1]) <= y_intersect and y_intersect <= max(segment1[1], segment2[1]):
          return True
    #print("CASE2FALSE")
    return False

def rectangle_intersection2(rotated_rectangle1:RotatedRectangle, rotated_rectangle2:RotatedRectangle) -> bool:
     
      rect1_vertvies = list(rotated_rectangle1.get_vertices())
      for x_2,y_2 in rotated_rectangle2.get_vertices():
        if is_point_in_rotated_rectangle(rect1_vertvies,x_2,y_2):
           return True
      return False


def cross_product(a, b, c):
    """
    Calculate the cross product of vectors AB and AC, where:
    - A is the starting point of the edge.
    - B is the ending point of the edge.
    - C is the point to check.

    Parameters:
        a (tuple): Coordinates of point A (x, y).
        b (tuple): Coordinates of point B (x, y).
        c (tuple): Coordinates of point C (x, y).

    Returns:
        float: Cross product value.
    """
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

def is_point_in_rotated_rectangle(vertices, x, y):
    """
    Check if a point (x, y) belongs to a rotated rectangle defined by its four vertices.

    Parameters:
        vertices (list): List of (x, y) tuples representing the rectangle's vertices.
        x (float): X-coordinate of the point to check.
        y (float): Y-coordinate of the point to check.

    Returns:
        bool: True if the point lies inside the rectangle, otherwise False.
    """
    # Initialize the sign of the cross product
    sign = None

    # Iterate through each edge of the rectangle
    for i in range(len(vertices)):
        a = vertices[i]
        b = vertices[(i + 1) % len(vertices)]  # Next vertex (wraps around to the first vertex)
        c = (x, y)

        # Calculate the cross product
        cp = cross_product(a, b, c)

        # Determine the sign of the cross product
        if cp == 0:
            # Point lies on the edge
            return True
        elif sign is None:
            # Set the initial sign
            sign = cp > 0
        elif (cp > 0) != sign:
            # Point is on the opposite side of at least one edge
            return False

    # Point is on the same side of all edges
    return True



# robot body and robot pose; robot body -- состоит из разных частей;  

#TODO подумать, как получить корректные координаты робота после поворота и соотв. коорд. центра
#TODO rectangle intersection problem

