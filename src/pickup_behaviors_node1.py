#!/usr/bin/env python

import py_trees
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import time

# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/check_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/check_object', Trigger)
            self.logger.debug(
                "  %s [CheckObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [CheckObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/check_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                self.blackboard.object_name = resp.message
                print("set to blackboard: ", resp.message)
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/check_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/get_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/get_object', Trigger)
            self.logger.debug(
                "  %s [GetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [GetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/get_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/get_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LetObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("object_delivered", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("object_delivered", access=py_trees.common.Access.READ)


    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/let_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/let_object', Trigger)
            self.logger.debug(
                "  %s [LetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [LetObject::setup() ERROR!]" % self.name)
        self.blackboard.object_delivered = 0

        

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/let_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                self.blackboard.object_delivered = self.blackboard.object_delivered + 1
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/let_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# TODO: Create any other required behavior like those to move the robot to a point, 
#       add or check elements in the blackboard, ...

# Behavior to move to blackboard.goal
class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MoveToGoal, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)
        self.distance_threshold = 0.15
        self.current_pose = [0,0]


    def setup(self):
        self.logger.debug("  %s [MoveToGoal::setup()]" % self.name)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odom)
        self.goal_sent = False


    def initialise(self):
        self.logger.debug("  %s [MoveToGoal::initialise()]" % self.name)

    def update(self):
        # Send the command to turtlebot
        if not self.goal_sent:
            goal = PoseStamped()
            goal.pose.position.x = self.blackboard.goal[0]
            goal.pose.position.y = self.blackboard.goal[1]
            self.goal_pub.publish(goal)
            self.goal_sent = True
        
        distance_to_goal = np.linalg.norm(self.blackboard.goal, self.current_pose)

        if distance_to_goal < self.distance_threshold:
            return py_trees.common.Status.RUNNING
        else:
            self.logger.debug("  %s [FindPickupSpot::update()]" % self.name)
            self.goal_sent = False
            return py_trees.common.Status.SUCCESS
            
    def terminate(self, new_status):
        self.logger.debug("  %s [MoveToGoal::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
    
    def get_odom(self, odom):
        self.current_pose = [odom.pose.pose.position.x, odom.pose.pose.position.y]

# Funtion to determine the pick up spot according to blackboard variables
class FindPickupSpot(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(FindPickupSpot, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("pickup_index", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("pickup_index", access=py_trees.common.Access.READ)
        self.blackboard.register_key("pickup_positions", access=py_trees.common.Access.READ)
        
        
    def setup(self):
        self.logger.debug("  %s [FindPickupSpot::setup()]" % self.name)
        self.blackboard.pickup_positions = [[1.25,0.5],[1.25,-1.25],[0.0,-1.25],[-0.5,1.25],[-1.25,0.5]]
        self.blackboard.pickup_index = 0
        
    def initialise(self):
        self.logger.debug("  %s [FindPickupSpot::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [FindPickupSpot::update()]" % self.name)
        try:
            # Extract the goal from pickup_positions list using current pickup_index
            self.blackboard.goal = self.blackboard.pickup_positions[self.blackboard.pickup_index]
            # Increment the pickup_index
            self.blackboard.pickup_index = self.blackboard.pickup_index + 1
            return py_trees.common.Status.SUCCESS
        except:
            rospy.logerr("[FindPickupSpot::update()]" % "Fail")
            return py_trees.common.Status.FAILURE

        
    def terminate(self, new_status):
        self.logger.debug("  %s [FindPickupSpot::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

# Funtion to determine the drop spot according to blackboard variables
class FindDropSpot(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(FindDropSpot, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("coke_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key("beer_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key("object_name", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [FindDropSpot::setup()]" % self.name)
        self.blackboard.coke_position = [1.5,1.5]
        self.blackboard.coke_position = [-1.5,-1.5]

        
    def initialise(self):
        self.logger.debug("  %s [FindDropSpot::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [FindDropSpot::update()]" % self.name)
        if self.blackboard.object_name == "beer":
            self.logger.debug("  %s [FindPickupSpot::update()]" % "Need to drop a beer")
            self.blackboard.goal = self.blackboard.beer_position
        else:
            self.logger.debug("  %s [FindPickupSpot::update()]" % "Need to drop a coke")
            self.blackboard.goal = self.blackboard.coke_position

        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.logger.debug("  %s [FindDropSpot::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    



if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    # Create Behaviors
    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    let_object = LetObject("let_object")
    move_to_goal = MoveToGoal("move_to_goal")
    find_pickup_spot = FindPickupSpot("find_pickup_spot")
    find_drop_spot = FindDropSpot("find_drop_spot")

    # go to pickup spot sequence
    go_to_seq = py_trees.composites.Sequence(name="go_to_seq", memory=True)
    go_to_seq.add_child(find_pickup_spot,move_to_goal,check_object)
    retry_pickup = py_trees.decorators.Retry(name="retry_pickup",child=go_to_seq, num_failures=6)

    

    root = py_trees.composites.Sequence(name="Pick and Place Object", memory=True)
    
    root.add_child(retry_pickup,get_object,find_drop_spot,move_to_goal,let_object)

    py_trees.display.render_dot_tree(root)
    print("Call setup for all tree children")
    root.setup_with_descendants() # call setup() of all behaviors in the tree (set up ROS topic, service ...)
    print("Setup done!\n\n")
    py_trees.display.ascii_tree(root)
    
    for _ in range(1000):
        root.tick_once() # untill the root return sth.
        time.sleep(1)