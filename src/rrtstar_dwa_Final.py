#!/usr/bin/python3
import numpy as np
import math
from matplotlib import collections as mc
import tf
import rospy
import random
from std_msgs.msg import String

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import ColorRGBA, Float64MultiArray, Header
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler



class Tree:
    
    def __init__(self, init_node, goal_node):
        self.init_node = init_node
        self.goal_node = goal_node
        self.nodes = [init_node]
        self.vertex_idx = {tuple(init_node): 0}
        self.edges = []
        self.distance = [0.0]
        self.goal_sample_rate = 0.05
           
    '''
        Method returning random state
    '''
    def RandomState(self, min_p, max_p): 
        checker = random.uniform(0,1)
        if checker >= self.goal_sample_rate:
            rand_x = random.uniform(min_p[0], max_p[0])
            rand_y = random.uniform(min_p[1], max_p[1])
        else:
             return self.goal_node
        return (rand_x, rand_y)
    '''
        Method returning the index idx of the new vertex
        Adds a new vertex (node) to the graph, if it does not exist
    '''
    def add_vertex(self, loc):
        if loc not in self.nodes:
            idx = len(self.nodes)
            self.nodes.append(loc)
            self.vertex_idx[loc] = idx
            self.distance.append(float('inf'))
        else:
            idx = self.vertex_idx[loc]  
        return idx
    '''
        Method adding edges to the graph
    '''
    def add_edges(self, new_idx, idx, distance):

        found_new_idx = [coord for coord in self.edges if coord[0] == new_idx]
        if not found_new_idx:
            self.edges.append((new_idx, idx))
            self.distance[new_idx] = self.distance[idx] + distance

        
    '''
        Method rewiring edges in the tree     
    '''
    def rewire_edge(self, neighbor_idx, new_parent_idx):

        replacement = (neighbor_idx, new_parent_idx)

        self.edges = [replacement if coord[0] == neighbor_idx else coord for coord in self.edges]

    '''
        Method returning path from init_node to goal_node
    '''
    def get_path(self, goal_node):
        
        path = []
        num = self.vertex_idx[ goal_node]

        while num != 0:
            found_edge = False
            for tup in self.edges:
                if tup[0] == num:
                    path.append(self.nodes[ num])
                    num = tup[1]
                    found_edge = True
                    break
            if not found_edge:
                break
        path.append(self.nodes[ 0])
        if len(path)==1:
            return []
        
        print("PATH =", list(reversed(path)))
        return list(reversed(path))
    

class RRTstar:
    '''
        Constructor.
        Initialize all the parameters for the path planning
    '''
    def __init__(self)-> None:
        self.last_map_time = rospy.Time.now()
        self.path = []
        self.obstacle_list = list()
        self.current_pose = ()
        self.current_pose_robot = ()
        self.iterations = 50
        self.env = []
        self.origin = []
        self.resolution = []
        self.temp =[]
        self.temp1 =[]
        self.temp2 =[]
        self.radius = 0.0
        self.collision_distance = 0.1  # the radius of the robot and check is in collision with the obsatcle
        self.is_unknown_valid = True #unknown space is considered valid
        self.delta = 0.50 # the thershold distance to generate the random new node
        self.gamma = 3.0 
        self.feedback_msg = []
        self.visual = Visual()
        self.MAX_LINEAR_VELOCITY = 0.2
        self.MAX_ANGULAR_VELOCITY = 1.0
        self.proximity_weight = 5.0  # Weight for proximity to obstacles
        self.reference_weight = 1.0  # Weight for closeness to reference trajectory
        self.target_heading_weight = 2.0  # Weight for target heading cost
        self.yaw = 0.0
        self.target = 0.0
        self.pmax = [0,0]
        self.last_processed_time = rospy.Time.now()
        # Subcriber the needed topic 
        self.odom = rospy.Subscriber("kobuki/odom", Odometry, self.get_odom)
        self.move_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.get_goal)
        self.projected_map = rospy.Subscriber('/projected_map', OccupancyGrid, self.get_gridmap)
        self.planning_feedback_pub = rospy.Publisher('/planning_feedback', String, queue_size=10)
        rospy.Timer(rospy.Duration(0.5), self.controller)
   
    '''
        Odometry callback: Gets current robot pose and stores it into current_pose ........ (From SLAM Node)
    '''
    def get_odom(self,odom):
        _, _, self.yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                                odom.pose.pose.orientation.y,
                                                                odom.pose.pose.orientation.z,
                                                                odom.pose.pose.orientation.w])
        self.current_pose_robot = (odom.pose.pose.position.x, odom.pose.pose.position.y, self.yaw)
        current_pose = (odom.pose.pose.position.x, odom.pose.pose.position.y)
        self.current_pose = tuple(np.round(current_pose,3))
    '''
        Controller callback : calling the DWA function
    '''
    def controller(self, event):
        
        if len(self.path) > 0:
            
            if self.path[0] == self.path[-1]:
                thresh = 0.18
            else:
                thresh = 0.3

            # If current wait point reached with some tolerance move to next way point, otherwise move to current point
            if np.linalg.norm(np.array(self.path[0]) - np.array(self.current_pose[0:2])) < thresh:
                print("Position {} reached".format(np.array(self.path[0])))
                del self.path[0]
                if len(self.path) == 0 and np.linalg.norm(np.array(self.target) - np.array(self.current_pose[0:2])) < thresh:
                    print("Final position reached!")
                    self.feedback_msg = "success"
                    self.move_robot([0,0])
                else:
                    velocity = self.dwa_local_planner(np.array(self.path[0]))
                    self.move_robot(velocity)
                    self.feedback_msg = "running"
            else:
                
                velocity = self.dwa_local_planner(np.array(self.path[0]))
                self.move_robot(velocity)
                self.feedback_msg = "running"
            
        else:
            self.move_robot([0,0])
        self.feedback_msg = str(self.feedback_msg)
        self.planning_feedback_pub.publish(self.feedback_msg)
        

    '''
        Method returning the current pose (x,y)
    '''
    def get_current_pose(self):
        
        return self.current_pose
    '''
        Method returning the current pose robot (x,y,theta)
    '''
    def get_current_pose_robot(self):
        return self.current_pose_robot
    '''
        Method returning the postion in the world coordinates
    '''
    def cell2world(self,map_coor):

        # convert cell position to world coordinates. If position outside map return `[]` or `None`
        map_coor = np.array(map_coor, dtype=float)
        p = (map_coor * self.resolution) + self.origin
        
		
        return list(p)
    '''
        Method gridmap callback 
    '''
    def get_gridmap(self,gridmap):
        current_time = rospy.Time.now()
        time_diff = (current_time - self.last_processed_time).to_sec()

        if time_diff >= 1.0:  # Check if the time difference is at least 1 second
            self.last_processed_time = current_time
            self.env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            self.origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.resolution = gridmap.info.resolution
            self.pmax = self.cell2world([self.env.shape[0],self.env.shape[1]])

            # checking the validty of the path in the new updated grid map
            if len(self.path) > 0 :
                for i in range(len(self.path)-1):
                    if i < len(self.path)-1:
                        if not self.steerTo(self.path[i], self.path[i+1]):  
                            self.path = []
                            self.move_robot([0,0])
                            self.feedback_msg= "running"
                            self.main(self.get_current_pose(), self.target, self.iterations)  # replan path
                            
                    
    '''
        Method getting the goal position from exploration
    '''
    def get_goal(self,msg):  
        
        self.path=[]
        print("New goal received: ({}, {})".format(msg.pose.position.x, msg.pose.position.y))
        self.target = (msg.pose.position.x, msg.pose.position.y)
        self.target = tuple(np.round(self.target,5))
        print("self.get_current_pose()",self.get_current_pose())
        if len(self.target) != 0: 
            self.move_robot([0,0]) 
            self.feedback_msg= "running"                                                
            self.main(self.get_current_pose(), self.target, self.iterations)
    '''
        Method returning the map position from the world frame
    '''
    def __position_to_map__(self,p):
        # convert world position to map coordinates. If position outside map return `[]` or `None`
        map_coor = (np.array([p[0], p[1]]) - self.origin) / self.resolution
        # convert to integer
        map = np.floor(map_coor).astype(int)
        # Check if the cell position is within the map boundaries
        if map[0] < 0 or map[1] < 0 or map[0] >= self.env.shape[0] or map[1] >= self.env.shape[1]: 
            return None
        else:
            return map
    '''
        Method returning True if the given state is in collision and False otherwise.
        x = state to be checked
        dist = the radius to check for possible collision around the state x
    '''
    def IsInCollision(self,x,dist):
        s=np.zeros(2,dtype=np.float32).reshape(1,2)
        s[0,0]=x[0]
        s[0,1]=x[1]
        point2 = self.__position_to_map__([s[0,0],s[0,1]])
        if point2 is None or self.env[point2[0], point2[1]] > 30:
            return True
        x,y = point2[0], point2[1]
        dist = int(math.ceil(dist/ self.resolution))
        for i in range(-dist, dist+1):
            for j in range(-dist, dist+1):
                x_cor, y_cor = x+i, y+j
                if x_cor >= 0 and x_cor < self.env.shape[0] and y_cor >= 0 and y_cor < self.env.shape[1]:
                    if self.env[x_cor, y_cor] >= 30:
                        return True
                    if self.env[x_cor, y_cor] == -1 and not self.is_unknown_valid:
                        return True
                else:
                    if not self.is_unknown_valid:
                        return True

        return False
    '''
        Method returning the euclidean distance from two points 
    '''     
    def euclidean(self,p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    '''
        Method returning the nearest vertex, index and minimum distance to the random node
    '''
    def NearestState(self,rand_node, T):
        near_node = 0
        near_idx = 0
        min_dist = float("inf")
        
        #checking this node is obstacle or not
        if not self.IsInCollision(rand_node,0.1):
            for idx, v in enumerate(T.nodes):
                #calculates the Euclidean distance between the vertex and the rand_node
                edist = self.euclidean(v, rand_node)
                if edist < min_dist:
                    min_dist = edist
                    near_node = v
                    near_idx = idx
            return near_node, near_idx, min_dist
 
    '''
        Method returning the near neighbors within a given radius
    '''
    def find_near_neighbors(self,new_node, gamma, T):
        near_neighbors = []
        for idx, node in enumerate(T.nodes):
            if self.euclidean(new_node, node) <= gamma and self.steerTo(new_node, node) == 1:
                near_neighbors.append((node, idx))
        return near_neighbors
    '''
        Method returning new vertex in the direction of the line connecting the random and near vertices
    '''
    def NewState(self,near_node, rand_node, delta, min_dist):
        if (min_dist >= delta):
            D = min_dist
            d = min(D, delta)
            new_x = near_node[0] + (d * (rand_node[0] - near_node[0]) / D)
            new_y = near_node[1] + (d * (rand_node[1] - near_node[1]) / D)
            new_node = new_x, new_y
            if not self.IsInCollision(new_node,0.14): 
                return (new_node)
        else:
            return rand_node
    '''
        Method generating the path with RRT* algorithm
    '''
    def generateRRT_star(self,init_node, goal_node, iterations, delta, gamma):
        goal_found = False
        T = Tree(init_node, goal_node)
        # distance threshold for choosing new state
        beta = 0.50
        for i in range(iterations):
            rand_node = T.RandomState(self.origin,self.pmax)

            nearest_node_data = self.NearestState(rand_node, T)
            if nearest_node_data == None:
                continue
            
            if nearest_node_data:
                near_node, near_idx, min_dist = nearest_node_data
                
                new_node = self.NewState(near_node, rand_node, beta, min_dist)
                
                
                if new_node is None or not self.steerTo(new_node, near_node):
                    continue
                
                proximal_node = near_node
            
                new_idx = T.add_vertex(new_node)
                distance = self.euclidean(new_node, proximal_node)
                
                T.add_edges(new_idx, near_idx, distance)
                dist = self.euclidean(new_node, T.goal_node)
                
                # check if goal node is near to the new node and they can be connected in straight line
                if 0< dist < delta: 
                    
                    if self.steerTo(new_node, goal_node): 
                        x_goalidx = T.add_vertex(T.goal_node)
                        print("Goal saved in dictionary",T.vertex_idx[goal_node])
                        goal_found= True
                        T.add_edges( x_goalidx, new_idx,dist)

                new_cost = T.distance[new_idx] 
            
                near_neighbors = self.find_near_neighbors(new_node, gamma, T)
                for neighbor_node, neighbor_idx in near_neighbors:
                    neigh_cost = new_cost + self.euclidean(new_node, neighbor_node)
                    if neigh_cost < T.distance[neighbor_idx]:
                        
                        T.distance[neighbor_idx] = neigh_cost
                        T.rewire_edge(neighbor_idx, new_idx)

        temp = float('inf')
        index= None
        # if goal not found try to connect the goal node from the neighbouring nodes (lowest cost connection) already in the tree 
        if not goal_found:
            near_neighbors = self.find_near_neighbors(T.goal_node, 10, T)
        
            for neighbor_node, neighbor_idx in near_neighbors:
                    neigh_cost = T.distance[neighbor_idx] + self.euclidean(T.goal_node, neighbor_node)
                    if neigh_cost < temp:
                        index = neighbor_idx
                        temp = neigh_cost
            x_goalidx = T.add_vertex(T.goal_node)
            if index!= None:
                T.add_edges( x_goalidx, index,self.euclidean(T.goal_node, T.nodes[index]))

        return T
    '''
        Method returning 1 if two points can be connected with straight line, 0 otherwise
    '''
    def steerTo (self,start, end):
        DISCRETIZATION_STEP=0.01
        dists=np.zeros(2,dtype=np.float32)
        for i in range(0,2): 
            dists[i] = end[i] - start[i]

        distTotal = 0.0
        for i in range(0,2): 
            distTotal =distTotal+ dists[i]*dists[i]

        distTotal = math.sqrt(distTotal)
        if distTotal>0:
            incrementTotal = distTotal/DISCRETIZATION_STEP
            for i in range(0,2): 
                dists[i] =dists[i]/incrementTotal
            numSegments = int(math.floor(incrementTotal))
            stateCurr = np.zeros(2,dtype=np.float32)
            for i in range(0,2): 
                stateCurr[i] = start[i]
            for i in range(0,numSegments):
                if self.IsInCollision(stateCurr,0.14):
                    return 0
                for j in range(0,2):
                    stateCurr[j] = stateCurr[j]+dists[j]
            if self.IsInCollision(end,0.14):
                return 0
        return 1
    '''
        Lazy vertex contraction make the path beocme straight line by using steerTo
        Method simplifying a given path by removing any unnecessary intermediate vertices that don't affect the path's validity.
    '''
    def lvc(self,path):
        
        for i in range(0,len(path)-1):
            for j in range(len(path)-1,i+1,-1):
                ind=0
                ind=self.steerTo(path[i],path[j])
                # print("ind",ind)
                if ind==1:
                    pc=[]
                    for k in range(0,i+1):
                        pc.append(path[k])
                    for k in range(j,len(path)):
                        pc.append(path[k])
                    return self.lvc(pc)
               
        return path
    '''
        Method returning the linear and angular velocity from DWA algorithm
    '''
    def dwa_local_planner(self, goal_pose):
        best_velocity = None
        best_cost = float('inf')
        # Generate a set of dynamically feasible velocities
        for v in np.linspace(0, self.MAX_LINEAR_VELOCITY, 10):
            for w in np.linspace(-self.MAX_ANGULAR_VELOCITY, self.MAX_ANGULAR_VELOCITY, 50):
                velocity = (v, w)

                # Predict the resulting point, add the for loop to make the list of trajectory 
                dt=2
                num_steps=10
                trajectory = self.predict_trajectory(velocity, dt, num_steps)
                
                self.visual.publish_trajectory(trajectory)
                # Calculate the target heading based on the goal of robot
                target_heading = self.calculate_target_heading(goal_pose, trajectory)
                
                # Compute the cost of the next point
                cost = self.compute_cost(trajectory, goal_pose, target_heading)
                
                # Select the best velocity that minimizes the cost
                if cost < best_cost:
                    best_cost = cost
                    best_velocity = velocity
                    best_traject = trajectory
        self.visual.publish_best_trajectory(best_traject)
        return best_velocity
    '''
        Method wraping angle 
    '''
    def wrap_angle(self,angle):
        return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
    '''
        Method returning the target heading of the robot
    '''
    def calculate_target_heading(self,goal_pose, trajectory):
        x, y = goal_pose 
        px, py, p_theta = trajectory[-1]
        xr, yr, theta_r = self.get_current_pose_robot()
        
        # Calculate the angle of the target point relative to the robot's position
        dx1 = x - xr
        dy1 = y - yr
        angle1 = math.atan2(dy1, dx1)

        dx2 = px - xr
        dy2 = py - yr
        angle2 = math.atan2(dy2, dx2)

        target_angle = angle1-angle2
        
        # Calculate the target heading as the difference between the target angle and the robot's heading direction
        target_heading = self.wrap_angle(target_angle)

        return target_heading

    '''
        Method returning the predicted trajectory of the robot in dt=2sec time step with a resolution of num_steps=10
    '''
    def predict_trajectory( self, velocity, dt, num_steps):
        robot_state= self.get_current_pose_robot()
        x, y, theta = robot_state
        v, w = velocity

        trajectory = []

        # Predict the next state based on velocity
        for _ in range(num_steps):
            x += v * math.cos(theta) * (dt/num_steps)
            y += v * math.sin(theta) * (dt/num_steps)
            theta += w * (dt/num_steps)
            trajectory.append((x,y,theta))

        return trajectory
    '''
        Method returning the cost by intergrating the obstacle cost, heading cost and reference cost
    '''
    def compute_cost(self,trajectory, goal_pose, target_heading):
        
        # Compute proximity to obstacles
        obstacle_cost = 0.0
        for point in trajectory:
            if self.IsInCollision([point[0], point[1]],0.16):
                obstacle_cost += 5.0

        # Compute closeness to reference trajectory
        reference_cost = 0.0
       
        for i in range(len(trajectory)):
            ref_x, ref_y = goal_pose
            traj_x, traj_y, _ = trajectory[i]
            distance = math.sqrt((ref_x - traj_x) ** 2 + (ref_y - traj_y) ** 2)
            reference_cost += distance
          
        # Compute target heading cost
        target_heading_cost = abs(target_heading)

        # Compute the overall cost
        cost = (
            self.proximity_weight * obstacle_cost
            + self.reference_weight * reference_cost
            + self.target_heading_weight * target_heading_cost
        )

        return cost
    '''
        Method returning True or False if the robot reach the goal position or not
    '''
    def reached_goal(self,current_pose_robot, goal_pose):
        tolerance = 0.05  # Tolerance for considering the goal reached
        # Check if the current robot pose is close enough to the goal pose
        distance = math.sqrt((current_pose_robot[0] - goal_pose[0]) ** 2 + (current_pose_robot[1] - goal_pose[1]) ** 2)
        if distance < tolerance:
            return True

        return False
    '''
        Method returning True or False if the robot reach the target position or not
        Note that, target points are the intermidate points in the path and Goal point is the final point in the path.
    '''
    def reached_target(self,current_pose_robot, goal_pose):
        tolerance = 0.2  # Tolerance for considering the target reached
        # Check if the current robot pose is close enough to the goal pose
        distance = math.sqrt((current_pose_robot[0] - goal_pose[0]) ** 2 + (current_pose_robot[1] - goal_pose[1]) ** 2)
        if distance < tolerance:
            return True

        return False
    '''
        Method executing the control actions based on the given velocity
    '''
    def move_robot(self,velocity):
        # Move the robot according to the velocity
        self.visual.__send_commnd__(velocity[0],velocity[1])
        
    '''
        Method generate the path using the RRT* (self.generateRRT_star... function)
    '''
    def main(self,init_node,goal_node, iterations):
        trail =0
        RRT = self.generateRRT_star(init_node, goal_node,  iterations, self.delta, self.gamma)
        self.path = RRT.get_path(goal_node)
        self.path = self.lvc(self.path)
        self.visual.publish_path(self.path, self.get_current_pose())
        # replan with different number of iterations if path not found
        while self.path==[] and trail<3:
            self.feedback_msg = "running"
            trail =trail + 1
            iterations = iterations * 2
            RRT = self.generateRRT_star(init_node, goal_node,  iterations, self.delta, self.gamma)
            self.path = RRT.get_path(goal_node)
            self.path = self.lvc(self.path)
            self.visual.publish_path(self.path, self.get_current_pose())
            if len(self.path)==0 and trail<3:
                print("Replanning ...")
            
        if self.path==[]:
            print("Path NOT Found!")
            self.feedback_msg = "failure"
    

class Visual:
    def __init__(self)-> None: 
        self.cmd_pub = rospy.Publisher('/kobuki/commands/wheel_velocities', Float64MultiArray, queue_size=10)
        self.marker_pub = rospy.Publisher('path_marker', Marker, queue_size=1)
        self.dwa_pub = rospy.Publisher('/dwa', Path, queue_size=10)
        self.dwa_best_traject_pub = rospy.Publisher('/best_dwa_traject', Path, queue_size=10)
    '''
        Method transforming linear and angular velocity (v, w) into a Twist message and publish it
    '''
    def __send_commnd__(self,v, w):
        v_max = 0.1 # max linear velocity
        w_max = 1   # max angular velocity
        v = np.clip(v, -v_max, v_max)
        w = np.clip(w, -w_max, w_max)

        wheel_base = 0.23
        wheel_radius = 0.035

        left_lin_vel =  (2*v + w*wheel_base) / (2*wheel_radius)
        right_lin_vel = (2*v - w*wheel_base) / (2*wheel_radius)

        cmd = Float64MultiArray()
        cmd.data = [left_lin_vel, right_lin_vel]
        self.cmd_pub.publish(cmd)
    '''
        Method publishing the path 

    '''
    def publish_path(self,path,pose):
        if len(path) > 1:
            # print("Publish path!")
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)

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
            p.x = pose[0]
            p.y = pose[1]
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
    '''
        Method publishing the dwa path
    '''
    def publish_trajectory(self,trajectory):
        if len(trajectory) > 1:
        
            path = Path()
            path.header = Header()
            path.header.frame_id = 'world'
            path.header.stamp = rospy.Time.now()

            for state in trajectory:
                q = quaternion_from_euler(0, 0, state[2])
                pose = PoseStamped()
                pose.header.frame_id = 'world'
                pose.header.stamp = rospy.Time.now()

                pose.pose.position.x = state[0]
                pose.pose.position.y = state[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                path.poses.append(pose)

            self.dwa_pub.publish(path)

    '''
        Method publishing the best trajectory
    '''
    def publish_best_trajectory(self,trajectory):
        if len(trajectory) > 1:
        
            path = Path()
            path.header = Header()
            path.header.frame_id = 'world'
            path.header.stamp = rospy.Time.now()

            for state in trajectory:
                q = quaternion_from_euler(0, 0, state[2])
                pose = PoseStamped()
                pose.header.frame_id = 'world'
                pose.header.stamp = rospy.Time.now()

                pose.pose.position.x = state[0]
                pose.pose.position.y = state[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                path.poses.append(pose)

            self.dwa_best_traject_pub.publish(path)

if __name__ == '__main__':
    rospy.init_node("RRT_star_DWA")
    RRTstar_DWA = RRTstar()
    rospy.spin()


