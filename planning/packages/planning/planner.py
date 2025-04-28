import logging
import numpy as np
import networkx as nx
import random
import math

from typing import List,Union, Tuple

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    PlacedPrimitive,
    PlanningQuery,
    PlanningResult,
    PlanningSetup,
    PlanStep,
    Circle,
    Rectangle,
    SimulationResult,
    simulate,
)

from dataclasses import dataclass

from zuper_commons.logs import ZLogger

logger = ZLogger(__name__)
logger.setLevel(level = logging.DEBUG)
"""
logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO, handlers=[    
        logging.StreamHandler()
    ])
"""


__all__ = ["Planner"]


class Planner:
    params: PlanningSetup
    query_num:int = 0
    
    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: PlanningSetup):
        context.info("initialized")
        self.params = data

        # This is the interval of allowed linear velocity
        # Note that min_velocity_x_m_s and max_velocity_x_m_s might be different.
        # Note that min_velocity_x_m_s may be 0 in advanced exercises (cannot go backward)
        max_velocity_x_m_s: float = self.params.max_linear_velocity_m_s
        min_velocity_x_m_s: float = self.params.min_linear_velocity_m_s

        # This is the max curvature. In earlier exercises, this is +inf: you can turn in place.
        # In advanced exercises, this is less than infinity: you cannot turn in place.
        max_curvature: float = self.params.max_curvature

        # these have the same meaning as the collision exercises
        body: List[PlacedPrimitive] = self.params.body
        environment: List[PlacedPrimitive] = self.params.environment

        # these are the final tolerances - the precision at which you need to arrive at the goal
        tolerance_theta_deg: float = self.params.tolerance_theta_deg
        tolerance_xy_m: float = self.params.tolerance_xy_m

        # For convenience, this is the rectangle that contains all the available environment,
        # so you don't need to compute it
        bounds: Rectangle = self.params.bounds

    def on_received_query(self, context: Context, data: PlanningQuery):
        self.query_num += 1
        
        # A planning query is a pair of initial and goal poses
        start: FriendlyPose = data.start
        goal: FriendlyPose = data.target

        # You start at the start pose. You must reach the goal with a tolerance given by
        # tolerance_xy_m and tolerance_theta_deg.

        # You need to declare if it is feasible or not
        
        #TODO возможно, надо учесть габариты робота (пока без этого)
        
        #NOTE possible paramter for RTT algorithm
        logger.info(f"req num {self.query_num}: map bounds {self.params.bounds}")
        factor = 0
        rtt_alg = rtt(self.query_num,
                     start = start, 
                     goal = goal, 
                     robot_body=self.params.body,
                     obstacles = self.params.environment, 
                     rectangle_map = self.params.bounds, 
                     num_of_samples = 200, 
                     factor = 0, 
                     tolerance_xy = .05, 
                     max_linear_velocity_m_s = self.params.max_linear_velocity_m_s, 
                     max_angular_velocity_deg_s = self.params.max_angular_velocity_deg_s,
                     tolerance_theta_deg = self.params.tolerance_theta_deg,
                     check_start_to_goal_first = True)
        


        logger.info(f"req num {self.query_num}: rtt parameters: {rtt_alg.print_rtt_parameters()}")
        
        graph:nx.Graph = rtt_alg.build_graph()
        feasible = rtt_alg.is_goal_feasible(graph)
                
        if not feasible:
            # If it's not feasible, just return this.
            logger.info(f'req num {self.query_num}: plan is not feasible, leave.') 
            result: PlanningResult = PlanningResult(False, None)
            context.write("response", result)
            return

        # If it is feasible you need to provide a plan.

        # A plan is a list of PlanStep
        logger.info(f'req num {self.query_num}: plan is feasible, building plan') 
        plan: List[PlanStep] = rtt_alg.build_plan_steps_for_graph(graph)
        logger.info(f'req num {self.query_num}: plan is {plan}')
        #print(f'plan is {plan}')
        # A plan step consists in a duration, a linear and angular velocity.
       
        """
        # For now let's just trace a square of side L at maximum velocity.
        L = 1.0
        duration_straight_m_s = L / self.params.max_linear_velocity_m_s
        duration_turn_deg_s = 90.0 / self.params.max_angular_velocity_deg_s
        # The plan will be: straight, turn, straight, turn, straight, turn, straight, turn

        straight = PlanStep(
            duration=duration_straight_m_s,
            angular_velocity_deg_s=0.0,
            velocity_x_m_s=self.params.max_linear_velocity_m_s,
        )
        turn = PlanStep(
            duration=duration_turn_deg_s,
            angular_velocity_deg_s=self.params.max_angular_velocity_deg_s,
            velocity_x_m_s=0.0,
        )
           duration *   angular_velocity_deg_s = angular_velocity_deg
        plan.append(straight)
        plan.append(turn)
        plan.append(straight)
        plan.append(turn)
        plan.append(straight)
        plan.append(turn)
        plan.append(straight)
        plan.append(turn)
        """ 
        
        result: PlanningResult = PlanningResult(feasible, plan)
        context.write("response", result)







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

    def get_line_segments(self)-> List[Tuple[float,float]]:
        yield  (self.x1,self.y1), (self.x2,self.y2)
        yield  (self.x2,self.y2), (self.x3,self.y3)
        yield  (self.x3,self.y3), (self.x4,self.y4)
        yield  (self.x4,self.y4), (self.x1,self.y1)

    def get_corners(self)-> List[Tuple[float, float]] :
        #Need to compare segment sized and select two longest
        return [ (self.x1, self.y1), (self.x2,self.y2) , (self.x3,self.y3) , (self.x4, self.y4) ]




@dataclass
class RandomPoint:
     x: float
     y: float

     def  __str__(self):
        return f"Random point ({self.x},{self.y})"

#Algorithm BuildRRT
#    Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
#    Output: RRT graph G

#    G.init(qinit)
#    for k = 1 to K do
#        qrand ← RAND_CONF()
#        qnear ← NEAREST_VERTEX(qrand, G)
#        qnew ← NEW_CONF(qnear, qrand, Δq)
#        G.add_vertex(qnew)
#        G.add_edge(qnear, qnew)
#    return G

class rtt:
     
      def __init__(self, query_num:int, start:FriendlyPose, goal:FriendlyPose, obstacles:List[PlacedPrimitive],  
                         robot_body:  List[PlacedPrimitive], 
                         rectangle_map:Rectangle, 
                         num_of_samples:int, 
                         factor:float, 
                         tolerance_xy:float,
                         max_linear_velocity_m_s:float,
                         max_angular_velocity_deg_s:float,
                         tolerance_theta_deg:float,
                         check_start_to_goal_first:bool = False):
          
          self.query_num = query_num               
          self.start = start
          self.goal = goal
          self.robot_body = robot_body
          self.rectangle_map = rectangle_map
          self.num_of_samples = num_of_samples
          self.factor = factor
          self.obstacles: List[PlacedPrimitive] = obstacles
          self.body_tolerance = tolerance_xy
          self.max_linear_velocity_m_s = max_linear_velocity_m_s
          self.max_angular_velocity_deg_s = max_angular_velocity_deg_s
          self.tolerance_theta_deg = tolerance_theta_deg
          self.check_start_to_goal_first = check_start_to_goal_first

      def print_rtt_parameters(self):
        return f"tolerance_xy = {self.body_tolerance}, map bounds = x:{self.rectangle_map.xmin, self.rectangle_map.xmax} \
                                                     y:{self.rectangle_map.ymin, self.rectangle_map.ymax}  "





      def build_graph(self)-> nx.Graph:
        
        #NOTE ignore steering for now
        
        logger.info("req num {query_num}: graph: start building graph!")
        G = nx.Graph() 
        G.add_node(self.start)
        
        ## TODO вероятно, можно будет удалить, поскольку в цикле я все это проверяю.
        if self.check_start_to_goal_first and self._no_obstacles_between_nodes(self.start, self.goal):
             G.add_node( self.goal )
             G.add_edge( self.start, self.goal )
             logger.info("req num {self.query_num}: graph: found direct path between start and goal!")
             return G    


        for i in range(self.num_of_samples):
           
           if True:
            nearest_node = self._nearest_node_from_graph(self.goal, G)
            if (nearest_node is not None) and self._no_obstacles_between_nodes(nearest_node, self.goal):
                G.add_node(self.goal)
                G.add_edge(nearest_node, self.goal) 
                logger.info(f"req num {self.query_num}: graph: found direct path between nearest_node = {nearest_node} and goal = {self.goal}!")
                return G   
           
           
           logger.debug(f"req num {self.query_num}: Probing sample {i}, number of nodes {G.number_of_nodes()}, number of edges {G.number_of_edges()}")
           q_random_point: RandomPoint = self._sample_environment()
           logger.debug(f"req num {self.query_num}: Got random point {q_random_point}")
           q_random_point_f_pose = FriendlyPose(q_random_point.x, q_random_point.y, 0)
           nearest_node = self._nearest_node_from_graph(self.goal, G)
                      
           if self._points_distance(q_random_point_f_pose, self.goal) <= self.body_tolerance:
             
                           
              logger.info(f"req num {self.query_num}: Random point {q_random_point_f_pose} is close to goal {self.goal}")
                                    
              if self._no_obstacles_between_nodes(nearest_node, self.goal):
                 logger.info(f"req num {self.query_num}: graph: no obstacle between nearest node {nearest_node} in graph and goal {self.goal}. We are done.")
                 G.add_node( self.goal )
                 G.add_edge( nearest_node, self.goal )
                 
                 return G
                  
           else:
             
                if self._no_obstacles_between_nodes(nearest_node, q_random_point_f_pose):
                        logger.info(f"req num {self.query_num}: graph: no obstacle between nearest node {nearest_node} in graph and new random point {q_random_point_f_pose}.\
                         Adding new point to graph.")
                        G.add_node(q_random_point_f_pose)
                        G.add_edge(nearest_node, q_random_point_f_pose)
                     
        #check if there is still a pass
        
        nearest_node = self._nearest_node_from_graph(self.goal, G)
        if self._no_obstacles_between_nodes(nearest_node, self.goal):
           G.add_node(self.goal)
           G.add_edge(nearest_node, self.goal) 
        
        return G


      def is_goal_feasible(self, G:nx.Graph) -> bool:
         if G.has_node(self.goal):
             try:
               return  nx.has_path(G,self.start, self.goal)
             except nx.NetworkXNoPath as ex:
                    logger.error("req num {self.query_num}: Shortest path error", exc_info = ex)  
                    return False 
         logger.warning("req num {self.query_num}: graph: no self.goal node in graph ")    
         return False    

      
      def _points_distance(self, pnt1:FriendlyPose, pnt2: FriendlyPose) -> float:
           return ((pnt1.x - pnt2.x)**2 + (pnt1.y - pnt2.y)**2)**.5

      def _nearest_node_from_graph(self, random_point: Union[RandomPoint,FriendlyPose], G:nx.DiGraph)-> FriendlyPose:
        min_distance = float('inf')
        nearest_node:FriendlyPose = None
        for node, data in G.nodes(data=True):
            f_pose:FriendlyPose = node#data['f_pos']
            node_x, node_y, _ = vars(f_pose).values()  #f_pose
            dist = ((node_x - random_point.x)**2 + (node_y -random_point.y)**2)**0.5
            if dist < min_distance:
                min_distance = dist
                nearest_node = f_pose     
        return nearest_node         

      
      def _sample_environment(self):
            x = (random.uniform( self.rectangle_map.xmin, self.rectangle_map.xmax ))
            y = (random.uniform( self.rectangle_map.ymin, self.rectangle_map.ymax ))
            return RandomPoint(x, y)

      def _no_obstacles_between_nodes(self, new_point:FriendlyPose, nearest_node:FriendlyPose) -> bool:
        
        x1, y1 = new_point.x, new_point.y
        x2, y2 = nearest_node.x, nearest_node.y
        
        theta_deg = angle_between_points(x1, y1, x2, y2)
        
        temp_robot_body_pos1 = PlacedPrimitive( FriendlyPose( x1, y1, theta_deg ), self.robot_body[0].primitive)
        primitive_rectangle_pos1  =  get_primitive_bounded_rectangle2(temp_robot_body_pos1)

        temp_robot_body_pos2 = PlacedPrimitive( FriendlyPose( x2, y2, theta_deg ), self.robot_body[0].primitive)
        primitive_rectangle_pos2  =  get_primitive_bounded_rectangle2(temp_robot_body_pos2)

        
        corners_pos1_lst: List[Tuple[float, float]]  = primitive_rectangle_pos1.get_corners()   
        corners_pos2_lst: List[Tuple[float, float]]  = primitive_rectangle_pos2.get_corners()   
        
        for corner_i in range(0,4):

          
          corner_i1_x, corner_i1_y = corners_pos1_lst[corner_i]#x1, y1 
          corner_i2_x, corner_i2_y = corners_pos2_lst[corner_i]#x2, y2 
          logger.info(f"req num {self.query_num}: corner pos1 {corner_i} x,y = {corner_i1_x},{corner_i1_y}")
          logger.info(f"req num {self.query_num}: corner pos2 {corner_i} x,y = {corner_i2_x},{corner_i2_y} ")


          for obstacle in self.obstacles:
              if isinstance(obstacle.primitive, Circle) :
                 if line_segment_circle_intersection(corner_i1_x, corner_i1_y, corner_i2_x, corner_i2_y, obstacle.pose.x, obstacle.pose.y, obstacle.primitive.radius):
                    return False
                
              if isinstance(obstacle.primitive, Rectangle):
                rotated_rectangle = get_primitive_bounded_rectangle2(obstacle)
                if line_segment_rotated_rectangle_intersection(corner_i1_x, corner_i1_y, corner_i2_x, corner_i2_y, rotated_rectangle = rotated_rectangle):
                   return False

        
        return True    

      def build_plan_steps_for_graph(self, G:nx.Graph) -> List[PlanStep]:
           
           result:List[PlanStep] = []
           
           path = nx.shortest_path(G, self.start, self.goal)
           prev_x, prev_y, prev_angle_deg = vars(self.start).values()
           logger.info(f"req num {self.query_num}: shortest path contains {len(path)} nodes")
           
           for cur_node in path[1:]: 
                
                cur_x, cur_y, _ = vars(cur_node).values()  
                dx, dy, target_angle_deg, delta_theta = compute_relative_pose( prev_x, prev_y, prev_angle_deg, cur_x, cur_y)
                logger.debug(f'req num {self.query_num}: moving from {(prev_x, prev_y, prev_angle_deg)} to {(dx, dy, target_angle_deg)} by correcting angle to {delta_theta}') 
                
    

                turn_to_current = PlanStep(
                    duration = delta_theta / self.max_angular_velocity_deg_s,
                    angular_velocity_deg_s = self.max_angular_velocity_deg_s,
                    velocity_x_m_s = 0.0,
                )
                
                L = (dx**2 +dy**2)**.5
                move_to_current = PlanStep(
                    duration = L / self.max_linear_velocity_m_s,
                    angular_velocity_deg_s = 0.0,
                    velocity_x_m_s = self.max_linear_velocity_m_s,                               
                )
                result.append(turn_to_current)
                result.append(move_to_current)

                prev_x, prev_y, prev_angle_deg = cur_x, cur_y, target_angle_deg
         
           
           if abs( prev_angle_deg - self.goal.theta_deg ) > self.tolerance_theta_deg:
            
                if prev_angle_deg > self.goal.theta_deg :
                    final_theta_deg = 360 - (prev_angle_deg - self.goal.theta_deg)
                else:
                    final_theta_deg = (self.goal.theta_deg - prev_angle_deg)
                
                goal_final_turn = PlanStep(
                            duration = final_theta_deg / self.max_angular_velocity_deg_s,
                            angular_velocity_deg_s=self.max_angular_velocity_deg_s,
                            velocity_x_m_s=0.0,
                        )
                    
                result.append(goal_final_turn) 
            
           return result     



def compute_relative_pose(x, y, theta_deg, x1, y1):
    """
    Compute destination pose (x1, y1, angle) from current pose (x, y, θ°) to target (x1, y1).
    
    Args:
        x, y: Current position.
        theta_deg: Current orientation in degrees.
        x1, y1: Target position.
    
    Returns:
        tuple: (x1, y1, target_angle_deg) where angle is normalized to [0, 360).
    """
    # Relative translation
    dx = x1 - x
    dy = y1 - y
    
    # Target angle calculation
    target_angle_rad = math.atan2(dy, dx)
    target_angle_deg = math.degrees(target_angle_rad) % 360
    
    # Relative rotation (for turning)
    current_angle_deg = theta_deg % 360
    #delta_angle = ((target_angle_deg - current_angle_deg + 180) % 360) - 180
    
    if current_angle_deg > target_angle_deg :
       delta_angle = 360 - ( current_angle_deg - target_angle_deg)
    else:
       delta_angle = ( target_angle_deg - current_angle_deg )
    
    return dx, dy,target_angle_deg, delta_angle


def compute_relative_pose2(x, y, current_angle_deg, x1, y1):
    """
    Compute the relative transformation (dx, dy, dtheta) from start pose to target point.
    
    Args:
        x, y, theta: Start pose (position and orientation in radians).
        x1, y1: Target point coordinates.
    
    Returns:
        tuple: (dx, dy, dtheta) relative transformation.
    """
    # Translation
    dx = x1 - x
    dy = y1 - y
    
    # Rotation angle to face (x1, y1) from start pose
    dtheta = math.atan2(dy, dx) 
        
    #dtheta = target_angle - theta #TODO need proper fix
    
    # Normalize angle to [-π, π]
    dtheta_rad = (dtheta + math.pi) % (2 * math.pi) - math.pi
    dtheta_degrees = math.degrees(dtheta_rad) % 360 

    if current_angle_deg > dtheta_degrees :
       target_angle = 360 - (current_angle_deg - dtheta_degrees)
    else:
       target_angle = (dtheta_degrees - current_angle_deg)


    return dx, dy, target_angle


def angle_between_points(x1, y1, x2, y2):
    """
    Calculate the angle (in degrees) between two points (x1,y1) and (x2,y2).
    The angle is measured from the positive x-axis to the line connecting the points.
    
    Args:
        x1, y1: Coordinates of the first point.
        x2, y2: Coordinates of the second point.
    
    Returns:
        float: Angle in degrees [0, 360).
    """
    if x1 == x2:  # Vertical line
        if y2 > y1:
            return 90.0  # Pointing upwards
        else:
            return 270.0  # Pointing downwards
    else:
        dx = x2 - x1
        dy = y2 - y1
        angle_rad = math.atan2(dy, dx)  # Angle in radians
        angle_deg = math.degrees(angle_rad)  # Convert to degrees
        return angle_deg % 360  # Normalize to [0, 360)


def line_segment_circle_intersection(x1, y1, x2, y2, x, y, R):
    """
    Check if a line segment intersects a circle.

    Args:
        x1, y1: Start point of the line segment.
        x2, y2: End point of the line segment.
        x, y: Center of the circle.
        R: Radius of the circle.

    Returns:
        bool: True if the line segment intersects the circle, False otherwise.
    """
    # Vector from start to end of the segment
    dx = x2 - x1
    dy = y2 - y1

    # Vector from start of segment to circle's center
    fx = x - x1
    fy = y - y1

    # Compute dot product to find the projection of (fx, fy) onto the segment
    dot_product = fx * dx + fy * dy
    segment_length_squared = dx * dx + dy * dy

    # Clamp t to [0, 1] to ensure it lies on the segment
    if segment_length_squared == 0:  # Segment is a point
        t = 0
    else:
        t = max(0, min(1, dot_product / segment_length_squared))

    # Closest point on the segment to the circle's center
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    # Distance between closest point and circle's center
    distance_squared = (x - closest_x)**2 + (y - closest_y)**2

    # Check if distance <= radius
    return distance_squared <= R**2


def line_segment_rotated_rectangle_intersection(x1:float, y1:float, x2:float, y2:float, rotated_rectangle:RotatedRectangle):
     
    segment = (x1,y1),(x2,y2) 
    for rr_segment in  rotated_rectangle.get_line_segments():
       is_intersect = segments_intersect(segment, rr_segment)
       if is_intersect:
           return True  
    
    return False



def segments_intersect(segment1, segment2):
    """
    Check if two line segments intersect.
    
    Args:
        segment1: Tuple of ((x1, y1), (x2, y2)) for first segment
        segment2: Tuple of ((x3, y3), (x4, y4)) for second segment
        
    Returns:
        bool: True if segments intersect, False otherwise
    """
    (x1, y1), (x2, y2) = segment1
    (x3, y3), (x4, y4) = segment2
    
    # Calculate orientation for all possible triplets
    def orientation(a, b, c):
        val = (b[0] - a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
        if val == 0: return 0  # Colinear
        return 1 if val > 0 else 2  # Clockwise or counter-clockwise
    
    o1 = orientation((x1,y1), (x2,y2), (x3,y3))
    o2 = orientation((x1,y1), (x2,y2), (x4,y4))
    o3 = orientation((x3,y3), (x4,y4), (x1,y1))
    o4 = orientation((x3,y3), (x4,y4), (x2,y2))
    
    # General case - segments are not colinear
    if o1 != o2 and o3 != o4:
        return True
    
    # Special cases - segments are colinear
    # Check if any endpoint lies on the other segment
    def on_segment(a, b, c):
        """Check if point c lies on segment ab"""
        return (min(a[0], b[0]) <= c[0] <= max(a[0], b[0]) and
                min(a[1], b[1]) <= c[1] <= max(a[1], b[1]))
    
    if o1 == 0 and on_segment((x1,y1), (x2,y2), (x3,y3)):
        return True
    if o2 == 0 and on_segment((x1,y1), (x2,y2), (x4,y4)):
        return True
    if o3 == 0 and on_segment((x3,y3), (x4,y4), (x1,y1)):
        return True
    if o4 == 0 and on_segment((x3,y3), (x4,y4), (x2,y2)):
        return True
    
    return False

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
    
    return RotatedRectangle( rotated_vertices[0][0], rotated_vertices[0][1], rotated_vertices[1][0], rotated_vertices[1][1],
    rotated_vertices[2][0], rotated_vertices[2][1], rotated_vertices[3][0], rotated_vertices[3][1] )


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



