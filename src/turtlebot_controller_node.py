#!/usr/bin/python3

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from geometry_msgs.msg import PoseStamped

import time


# Wrap angle between -pi and pi
def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

# Pahh planning
def compute_path(start_p, goal_p, state_validity_checker, bounds, max_time=1.0):
    if goal_p[0] < bounds[0] or goal_p[0] > bounds[1] or goal_p[1] < bounds[2] or goal_p[1] > bounds[3] : 
        return []

    rrt = RRT(state_validity_checker, dominion=bounds, max_time=max_time)
    path = rrt.compute_path(start_p, goal_p)

    if path:
        return path
    else:
        return []

# Controller
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    """ Computes the control command to move from current position to goal. """
    theta_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    w = Kw * wrap_angle(theta_d - current[2])
    v = 0
    if abs(w) < 0.2: # to avoid move while turning
        v = Kv * np.linalg.norm(goal - current[0:2])
    return v, w

class Controller:
    def __init__(self, odom_topic, cmd_vel_topic, distance_threshold):

        # Attributes
        self.distance_threshold = distance_threshold                                                # Distance threshold to way point
        self.current_pose = None                                                                    # Current robot SE2 pose                           
        self.goal = None                                                                            # A goal is set
        self.path = []                                                                              # List of points which define the plan. None if there is no plan
        # Parameters
        self.Kv = 0.5                   # Proportional linear velocity controller
        self.Kw = 0.5                   # Proportional angular velocity controller
        self.v_max = 0.35               # Maximum linear velocity control action
        self.w_max = 0.3                # Maximum angular velocity control action

        # Publishers
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom)
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)
        
        # Timers
        rospy.Timer(rospy.Duration(0.1), self.controller)
    
    # Odometry callback
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    
    # Goal callback
    def get_goal(self, goal):
        if self.current_pose is not None:
            print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            self.path = None                                                    # to send zero velocity while planning
            self.path = [self.current_pose[0:2], self.goal]                     # to avoid path planning
            self.publish_path(self.path)
            del self.path[0]                                                    # remove current pose
        
    # Iterate: check to which way point the robot has to face. Send zero velocity if there's no active path.
    def controller(self, event):
        v = 0   
        w = 0
        if self.path is not None and len(self.path) > 0:
            
            # If current wat point reached with some tolerance move to next point otherwise move to current point
            if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < self.distance_threshold:
                print("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    print("Final position reached!")
            else:
                v, w = move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)
        self.__send_commnd__(v, w)
    

    # Publishers
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)
        
    def publish_path(self, path):
        if len(path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 1
            color_red.g = 0
            color_red.b = 0
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)


class Node:
    def __init__(self, pose, parent = None) -> None:
        self.pose = pose
        self.parent = parent

# Define RRT class (you can take code from Autonopmous Systems course!)
class RRT:
    def  __init__(self, state_validity_checker, max_time=10, delta_q=0.4, p_goal=0.6, dominion=[-10, 10, -10, 10]):
        # define constructor ...
        self.state_validity_checker = state_validity_checker
        self.max_time = max_time
        self.delta_q = delta_q
        self.p_goal = p_goal
        self.dominion = dominion
        
    def compute_path(self, q_start, q_goal):
        # Implement RRT algorithm.
        # Use the state_validity_checker object to see if a position is valid or not.q
        q_start =Node(q_start)
        q_goal = Node(q_goal)
        G = [] 
        E = []
        G.append(q_start)
        start_time = time.time()
        while (time.time()-start_time) < self.max_time:
            qrand = self.RAND_CONF(self.p_goal,q_goal)
            qnear = self.NEAREST_VERTEX(qrand,G)
            qnew = self.NEW_CONF(qnear, qrand, self.delta_q)
            if self.IS_SEGMENT_FREE(qnear, qnew):
                G.append(qnew)
                qnew.parent = qnear
                E.append([qnear,qnew])

                if (qnew.pose == q_goal.pose).all():
                    q_goal = qnew
                    path = self.FILL_PATH(q_start,q_goal)
                    smooth_path = self.SMOOTH_PATH(path)
                    return smooth_path
        return []
    
                             

    def RAND_CONF(self, p, qgoal):
        rand =  np.random.rand()
        if rand > p:
            qrand = (np.random.uniform(self.dominion[0],self.dominion[1]),np.random.randint(self.dominion[2],self.dominion[3]))# 
            while not self.state_validity_checker.is_valid(qrand):
                qrand = (np.random.uniform(self.dominion[0],self.dominion[1]),np.random.randint(self.dominion[2],self.dominion[3])) 

            return Node(pose=np.array(qrand)) 
        else:
            return qgoal

    def NEAREST_VERTEX(self, qrand, G):
        min_dist = 100000
               
        for i in range(len(G)):

            dist = self.dist(qrand,G[i])
            if dist < min_dist:
                min_dist = dist
                qnear = G[i]
        return qnear
    
    def NEW_CONF(self, qnear, qrand, delta_q):
        dist = self.dist(qnear,qrand)
        if delta_q > dist:
            return qrand 
        zeta = np.arctan2(qrand.pose[1] - qnear.pose[1], qrand.pose[0] - qnear.pose[0])
        new_pose  = (qnear.pose[0] + delta_q*math.cos(zeta),qnear.pose[1] + delta_q*math.sin(zeta))

        qnew = Node(np.array(new_pose))
        return qnew
    
    def IS_SEGMENT_FREE(self, qnear,qnew):
        if self.state_validity_checker.check_path([qnear.pose,qnew.pose]):  
            return True
        else:
            return False
    
    def FILL_PATH(self, qstart,qgoal):
        qcurrent = qgoal
        path = [qcurrent]
        
        while (qcurrent.pose != qstart.pose).any():   
            qcurrent = qcurrent.parent
            path.append(qcurrent) 
        path.reverse()
        return path
    
    def SMOOTH_PATH(self, path):
        current = path[-1]
        smooth_path = [current.pose]

        while (current.pose != path[0].pose).any():    
            for p in path:
                if self.IS_SEGMENT_FREE(current,p):
                    current = p
                    break
            smooth_path.append(p.pose)
        smooth_path.reverse()
        return smooth_path
    
    def dist(self, q1,q2):
        return ((q2.pose[0] - q1.pose[0])**2 + (q2.pose[1] - q1.pose[1])**2)**0.5


class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1, is_unknown_valid=True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data # np.array already
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        self.bound = [resolution*data.shape[0] ]

    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose): 
        if self.there_is_map == False:
            return(False)
        # TODO: convert world robot position to map coordinates using method __position_to_map__
        for x in np.arange(pose[0]-self.distance,pose[0]+self.distance,self.resolution):
            for y in np.arange(pose[1]-self.distance,pose[1]+self.distance,self.resolution):
                m = self.__position_to_map__(np.array([x,y]))

        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.

                if m is None:
                    if not self.is_unknown_valid:
                        return False
                elif self.map[m[0],m[1]]==-1:
                    if not self.is_unknown_valid:
                        return False
                elif self.map[m[0],m[1]]==100:
                    return(False)
                    
        return(True)

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path):
        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        step_size = 2*self.distance
        for i in range(len(path)-1):
            dist = np.linalg.norm(path[i] - path[i+1])
            zeta = np.arctan2(path[i+1][1] - path[i][1], path[i+1][0] - path[i][0])

            step = int(dist//step_size)

        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True.
        # For each point between 2 waypoints check if `is_valid`. 
            for j in range(step+1):
                inc_dist = step_size * (j)
                wp_check  = [path[i][0] + (inc_dist*math.cos(zeta)),path[i][1] + (inc_dist*math.sin(zeta))] 
                if not self.is_valid(wp_check):
                    #print("Path is not valid")
                    return False 
        # For the waypoint itself check if `is_valid`.
            wp_check = path[i+1]
            
            if not self.is_valid(wp_check):
               # print("Path is not valid")
                return False 
        return True
            
    def is_inside_map(self, p):
        return bool(self.__position_to_map__(p))

    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates. 
        
        # find p in image plane
        p_img  = p - self.origin
        
        # find row and col
        m = [int(np.floor(p_img[0] / self.resolution)),int(np.floor(p_img[1] / self.resolution))]
        # check that the cell is inside the grid map
        if m[0] >= self.map.shape[0] or m[1] >= self.map.shape[1] or m[0] < 0 or m[1] < 0:
            return None
        
        return m

if __name__ == '__main__':
    rospy.init_node('turtlebot_controller')   
    node = Controller('/odom', '/cmd_vel', 0.15)
    
    # Run forever
    rospy.spin()
