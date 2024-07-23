#!/usr/bin/env python3
import py_trees
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Bool
import tf
import time


'''
    Subclass of behavior tree, representing the exploration task.
'''
class Explore(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super(Explore, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.WRITE)
        rospy.Subscriber("/explration_view_point", PoseStamped, self.callback) 
        self.exploration_completed= False
        self.position=[]
        self.mission_completed= False
                     
    def setup(self):
        self.logger.debug("  %s [Exploration::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [Exploration::initialise()]" % self.name)
        
    def callback(self, msg):
        # Extract the x and y position from the message
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Save the (x, y) position as a list
        self.position = [x, y, 0]
        
        if msg.pose.position.x==123456.0:
            self.exploration_completed= True

    def update(self):

        # write the best viewpoint in the BB
        
        if self.exploration_completed:
            self.logger.debug("  %s [Exploration Completed!!!]" % self.name)
            self.mission_completed = True
            return py_trees.common.Status.FAILURE
        else:
            self.blackboard.location = self.position
            return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.logger.debug("  %s [Exploration::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
'''
    Subclass of behavior tree, representing the perception task.
'''
class Perceive(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Perceive, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.WRITE)
        rospy.Subscriber("/perceived_point", PoseStamped, self.callback) 
        
    def setup(self):
        self.logger.debug("  %s [Perception::setup()]" % self.name)
        
    
    def initialise(self):
        self.logger.debug("  %s [Perception::initialise()]" % self.name)
        self.position = []
        
        
    def callback(self, data):
        # Extract x, y, and yaw from the message
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        quat = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quat)
        yaw = euler[2]

        # Store the x, y, z, and yaw values in a list
        self.position = [x, y , z, yaw]

    def update(self):
        if len(self.position)>0:
            # write the pose of the detected object in the BB
            self.blackboard.location = self.position

        if len(self.position)>0:
            self.position =[]
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [No object Detected!!!]" % self.name)
            return py_trees.common.Status.FAILURE
        
        

    def terminate(self, new_status):
        self.logger.debug("  %s [Perception::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

'''
    Subclass of behavior tree, representing the motion planner.
'''
class Move_to(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Move_to, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.pub_move_robot = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/planning_feedback', String, self.callback)
        self.feedback = []
        self.publish_once = True
        
        
    def callback(self,msg):
        self.feedback = msg.data

    def setup(self):
        self.logger.debug("  %s [Move-to::setup()]" % self.name)
        
        
    def initialise(self):
        self.logger.debug("  %s [Move-to::initialise()]" % self.name)
        
    
    def update(self):
        if self.publish_once and self.blackboard.location!=[]:
            
            # read location from BB and publish it 
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world"
            msg.pose.position.x = self.blackboard.location[0]
            msg.pose.position.y = self.blackboard.location[1]
            msg.pose.orientation.w = 1.0

            self.pub_move_robot.publish(msg)
            self.publish_once = False

        if self.feedback == "failure":
            self.publish_once = True
            return py_trees.common.Status.FAILURE
        elif self.feedback == "success" or self.blackboard.location==[]:
            if self.feedback == "success":
                self.publish_once = True
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Move-to::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
'''
    Subclass of behavior tree, representing the picking task.
'''
class Pick_And_Place_1(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Pick_And_Place_1, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.pub = rospy.Publisher('end_effector_pose', PoseStamped, queue_size=10)
        rospy.Subscriber("end_effector_pose_reached", Bool, self.callback)

    def setup(self):
        self.logger.debug("  %s [Pick and Place::setup()]" % self.name)
        
    def initialise(self):
        self.end_effector_pose_reached = False
        self.publish_once = True
        self.count = 0
        self.logger.debug("  %s [Pick and Place::initialise()]" % self.name)
        
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    
    def update(self):
        self.count = self.count + 1 
        if self.publish_once:
            # read location from BB and publish it 
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world"
            msg.pose.position.x = self.blackboard.location[0]
            msg.pose.position.y = self.blackboard.location[1]
            msg.pose.position.z = self.blackboard.location[2]
            # convert yaw angle to quaternion and set the orientation
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            # publish the message
            self.pub.publish(msg)
            self.publish_once = False

        
        if not self.end_effector_pose_reached and self.count > 10:
            return py_trees.common.Status.FAILURE
        elif self.end_effector_pose_reached:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

'''
    Subclass of behavior tree, representing the placing task.
'''
class Pick_And_Place_2(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Pick_And_Place_2, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.pub = rospy.Publisher('end_effector_pose', PoseStamped, queue_size=10)
        rospy.Subscriber("end_effector_pose_reached", Bool, self.callback)

    def setup(self):
        self.logger.debug("  %s [Pick and Place::setup()]" % self.name)
        
    def initialise(self):
        self.end_effector_pose_reached = False
        self.publish_once = True
        self.count = 0
        self.logger.debug("  %s [Pick and Place::initialise()]" % self.name)
        
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    
    def update(self):
        self.count = self.count + 1 
        if self.publish_once:
            # read location from BB and publish it 
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world"
            msg.pose.position.x = self.blackboard.location[0]
            msg.pose.position.y = self.blackboard.location[1]
            msg.pose.position.z = self.blackboard.location[2]
            # convert yaw angle to quaternion and set the orientation
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            # publish the message
            self.pub.publish(msg)
            self.publish_once = False

        
        if not self.end_effector_pose_reached and self.count > 10:
            return py_trees.common.Status.FAILURE
        elif self.end_effector_pose_reached:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

'''
    Function of behavior tree, creating the behavior tree.
'''
def create_tree():
    global explore
    # Create Behaviors
    explore = Explore("Explore")
    perceive = Perceive("Perceive")
    move_to_explore = Move_to("Move_to_Explore")
    move_to_pick = Move_to("Move_to_Pick")
    move_to_place = Move_to("Move_to_Place")
    pick_1 = Pick_And_Place_1("Pick_1")
    pick_2 = Pick_And_Place_2("Pick_2")
    place_1 = Pick_And_Place_1("Place_1")
    place_2 = Pick_And_Place_2("Place_2")


    seq_1 = py_trees.composites.Sequence(name="Sequence", memory=True)
    seq_2 = py_trees.composites.Sequence(name="Sequence", memory=True)
    seq_3 = py_trees.composites.Sequence(name="Sequence", memory=False)
    seq_pick = py_trees.composites.Sequence(name="Sequence", memory=True)
    seq_place = py_trees.composites.Sequence(name="Sequence", memory=True)


    inverter = py_trees.decorators.Inverter(name="Inverter", child= perceive)
    

    seq_3.add_child(inverter) 
    seq_3.add_child(move_to_explore) 

    seq_1.add_child(explore)   
    seq_1.add_child(seq_3) 

    # building the right side of the tree     
    seq_pick.add_child(pick_1)   
    seq_pick.add_child(pick_2)

    seq_place.add_child(place_1)
    seq_place.add_child(place_2)

    seq_2.add_child(move_to_pick)
    seq_2.add_child(seq_pick)
    seq_2.add_child(move_to_place)
    seq_2.add_child(seq_place)

    # combining the left and right side of the tree
    selector = py_trees.composites.Selector(name="Selector", memory=True)
    selector.add_child(seq_1)
    selector.add_child(seq_2)

    return selector

'''
    Main function, executing the behavior tree.
'''
def run(it=1):
        if not explore.mission_completed:
            # py_trees.display.render_dot_tree(root)
            print("Call setup for all tree children")
            root.setup_with_descendants() 
            print("Setup done!\n\n")
            py_trees.display.ascii_tree(root)
            
            
            root.tick_once()
            time.sleep(1)
        else:
            print("mission accomplished!")
        
        
if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")
    root = create_tree()
    try:
        while not rospy.is_shutdown() :
            run()
    except KeyboardInterrupt:
        pass
    # Run forever
    rospy.spin()