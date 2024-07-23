#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped,Pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped 
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan, Imu
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from wrap_angle_final import *
from openThreeD_final import icp_register
import copy
from scipy.spatial import distance
from scipy.stats import chi2



class DifferentialDrive:
    def __init__(self) -> None:

        # robot constants
        self.left_wheel_radius = 0.035
        self.right_wheel_radius = 0.035
        self.wheel_base_distance = 0.230


        self.current_time = 0
        self.last_scan_pose = np.array([10,10,10]).reshape(-1,1)
        self.scans = [] # list to store scans
        self.scan_time_stamp=[]  # to store the time stamp of the scans
        self.dis_thresh = 1.0  # the minimum distance before which new scan should not be taken 
        self.overlap_thresh = 1.0
        self.scan_num = 150 # number of scans to store
        self.update_called = False  # flag to indicate if we are predicting or updating
        self.confidence_level = 0.99  # for individual compatibility test
        # Find the critical value
        self.critical_value = chi2.ppf(self.confidence_level, 3)

        # initial pose of the robot
        self.x = 3.0 
        self.y = -0.78 
        self.th = np.pi/2 

        self.X= np.array([[self.x], [self.y], [self.th]]).reshape(3,1)
        self.Xdeadreck = np.array([[self.x], [self.y], [self.th]]).reshape(3,1) # rebot state according to dead reckoning 
        self.XGroundTruth = np.array([[self.x], [self.y], [self.th]]).reshape(3,1) # ground truth robot state

        # initialize the map (state vector according to slam)
        self.Xk = np.array([self.x, self.y, self.th]).reshape(-1,1) 

        # initial robot pose uncertainty
        self.p_x=0.01
        self.p_y=0.01
        self.p_th=0.01
        self.P= np.array ([ [self.p_x**2, 0.0, 0.0], [0.0, self.p_y**2, 0.0], [0.0, 0.0, self.p_th**2]])

        # Initialize left and right wheel sensors
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.left_wheel_received = False
        self.right_wheel_received = False

        # wheel sensor uncertainty 
        self.left_wheel_sensor_noise = 0.6 #0.2 + 0.2
        self.right_wheel_sensor_noise = 0.6 #0.2 + 0.2
        self.Q = np.array([[ self.left_wheel_sensor_noise**2, 0],[0 ,self.right_wheel_sensor_noise**2]])


        # icp registration uncertainty 
        self.icpNoise_x = 0.1 #0.05 + 0.05
        self.icpNoise_y = 0.1 #0.05  + 0.05
        self.icpNoise_theta = 0.2
        self.R = np.array([[ self.icpNoise_x**2, 0, 0],[0, self.icpNoise_y**2, 0],[0, 0, self.icpNoise_theta**2]]).reshape(3,3)

        # IMU uncertainity
        self.imu_noise= np.array([0.0007**2]).reshape(1,1)

        

        # map covariance (SLAM)
        self.P_map = self.P # initialize it with robot uncertainty



        # Initialize linear velocity and angular velocity of the robot
        self.v = 0.0
        self.w = 0.0 


        

        self.last_time = rospy.Time.now()

        self.tf_br = TransformBroadcaster()
        

        # Lidar subscriber
        rospy.Subscriber('/kobuki/sensors/rplidar', LaserScan, self.lidar_callback, buff_size = 5)

        # imu subscriber
        rospy.Subscriber('/kobuki/sensors/imu', Imu, self.imu_callback)

        # Ground Truth Subscriber
        self.ground_tru_sub =  rospy.Subscriber("/kobuki/stonefish_simulator/ground_truth_robot_position", Odometry, self.GroundTruth_callback, buff_size = 15)

        # odom publisher
        self.odom_pub = rospy.Publisher("kobuki/odom", Odometry, queue_size=10)

        # joint state subscriber
        self.js_sub = rospy.Subscriber("/kobuki/joint_states", JointState, self.joint_state_callback, buff_size = 15)

        # Point Clouds publishers
        self.Grnd_truth_PC_pub = rospy.Publisher('Grnd_truth_pointCloud', PointCloud2, queue_size=10)
        self.slam_PC_pub = rospy.Publisher('slam_pointCloud', PointCloud2, queue_size=10)
        self.deadReck_PC_pub = rospy.Publisher('deadReck_pointCloud', PointCloud2, queue_size=10)

        # Initialize Marker Publisher of robot poses
        self.pose_pub = rospy.Publisher('poseMarkers', MarkerArray, queue_size=10)


        

        # # Initialize Path Publishers
        self.slam_pose_pub = rospy.Publisher('slam_Trajectory', Pose, queue_size=10) 
        self.Grnd_Truth_pose_pub = rospy.Publisher('Grnd_Truth_PATH', Pose, queue_size=10)
        self.DeadReck_pose_pub = rospy.Publisher('DeadReck_Path', Pose, queue_size=10)
        
        

        
        
           

    # Ground Truth Callback
    def GroundTruth_callback(self, GrndTrth):

        if self.update_called: 
            return
        
        # visualize the ground truth pose of the robot
        pose_gr = Pose()
        pose_gr.position.x = GrndTrth.pose.pose.position.x  # Set the position values
        pose_gr.position.y = GrndTrth.pose.pose.position.y
        pose_gr.position.z = 0.0
        # Set the orientation values
        
        pose_gr.orientation.x = GrndTrth.pose.pose.orientation.x
        pose_gr.orientation.y = GrndTrth.pose.pose.orientation.y
        pose_gr.orientation.z = GrndTrth.pose.pose.orientation.z
        pose_gr.orientation.w = GrndTrth.pose.pose.orientation.w

        self.Grnd_Truth_pose_pub.publish(pose_gr)

        self.XGroundTruth[0] = copy.deepcopy(GrndTrth.pose.pose.position.x)
        self.XGroundTruth[1] = copy.deepcopy(GrndTrth.pose.pose.position.y)
        
        
        euler_angels = euler_from_quaternion((GrndTrth.pose.pose.orientation.x, GrndTrth.pose.pose.orientation.y, GrndTrth.pose.pose.orientation.z, GrndTrth.pose.pose.orientation.w))
        self.XGroundTruth[2] = euler_angels[2]


    # Prediction is done here for the kalman filter
    def joint_state_callback(self, msg):  
        
        if msg.name[0] == "kobuki/wheel_left_joint":
            
            self.left_wheel_velocity = msg.velocity[0]
            self.left_wheel_received = True
            
        elif msg.name[0] == "kobuki/wheel_right_joint":    
            self.right_wheel_velocity = msg.velocity[0]
            self.right_wheel_received = True

        if self.left_wheel_received and self.right_wheel_received and not self.update_called:
            left_lin_vel = self.left_wheel_velocity * self.left_wheel_radius
            right_lin_vel = self.right_wheel_velocity * self.right_wheel_radius
            
            # computing the linear and angular velocities
            self.v = (left_lin_vel + right_lin_vel) / 2.0
            self.w = (left_lin_vel - right_lin_vel) / self.wheel_base_distance
            
            #calculate dt
            self.current_time = rospy.Time.from_sec(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
            dt = (self.current_time - self.last_time).to_sec()
            self.last_time = self.current_time


            # PREDICTION STEP OF KALMAN FILTER

            # integrate position
            
            self.X[0] = self.X[0] + np.cos(self.X[2]) * self.v * dt
            self.X[1] = self.X[1] + np.sin(self.X[2]) * self.v * dt
            self.X[2] = wrap_angle(self.X[2] + self.w * dt)
    
            self.q = quaternion_from_euler(0, 0, self.X[2][0])
            
            # update the robot pose in the map
            self.Xk [-3:]= copy.deepcopy(self.X)
            
            # visualize the trajectory of the robot according to the slam calculations
            pose_slam = Pose()
            pose_slam.position.x = self.X[0][0]  
            pose_slam.position.y = self.X[1][0]
            pose_slam.position.z = 0.0
            # Set the orientation values
            pose_slam.orientation.x = self.q[0]
            pose_slam.orientation.y = self.q[1]
            pose_slam.orientation.z = self.q[2]
            pose_slam.orientation.w = self.q[3]

            self.slam_pose_pub.publish(pose_slam)
            

            # Dead reckoning pose estimation
            self.Xdeadreck[0] = self.Xdeadreck[0] + np.cos(self.Xdeadreck[2]) * self.v * dt
            self.Xdeadreck[1] = self.Xdeadreck[1] + np.sin(self.Xdeadreck[2]) * self.v * dt
            self.Xdeadreck[2] = wrap_angle(self.Xdeadreck[2] + self.w * dt)

            # visualize the trajectory of the robot according to the Dead reckoning calculations
            pose = Pose()
            pose.position.x = self.Xdeadreck[0][0]  
            pose.position.y = self.Xdeadreck[1][0]
            pose.position.z = 0.0
            # Set the orientation values
            q = quaternion_from_euler(0, 0, self.Xdeadreck[2][0])
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            self.DeadReck_pose_pub.publish(pose)


            
            
            
            # Jacobians of the Motion Model

            # J1_X = [del fx / del x, del fx / del y, del fx / del theta]
            J1_X = np.array([1, 0, -self.v*np.sin(self.X[2][0])*dt]) 
            #J2_X = [del fx / del left wheel sensor noise , del fx / del right wheel sensor noise]
            J2_X = np.array([ 0.5*self.left_wheel_radius*dt*np.cos(self.X[2][0]) , 0.5*self.right_wheel_radius*dt*np.cos(self.X[2][0]) ]) 
            # J1_Y = [del fy / del x, del fy / del y, del fy / del theta]
            J1_Y = np.array([0, 1, self.v*np.cos(self.X[2][0])*dt])   
            #J2_Y = [del fy / del left wheel sensor noise , del fy / del right wheel sensor noise]
            J2_Y = np.array([ 0.5*self.left_wheel_radius*dt*np.sin(self.X[2][0]) , 0.5*self.right_wheel_radius*dt*np.sin(self.X[2][0]) ])
            # J1_theta = [del ftheta / del x, del ftheta / del y, del ftheta / del theta]
            J1_theta = np.array([0, 0, 1]) 
            #J2_theta = [del ftheta / del left wheel sensor noise , del ftheta / del right wheel sensor noise]
            J2_theta = np.array([ -1*self.left_wheel_radius*dt / self.wheel_base_distance , self.right_wheel_radius*dt / self.wheel_base_distance ])

            

            J1 = np.stack((J1_X, J1_Y, J1_theta), axis=-1).T 
            J2 = np.stack((J2_X, J2_Y, J2_theta), axis=-1).T
            
            
            # Covariance compounding
            self.P = J1 @ self.P @ J1.T + J2 @ self.Q @ J2.T

            # update the map covariance because the robot has moved
            self.P_map[-3:,-3:]= self.P # updating the current robot state covariance 

            # updating the cross covariances of the robot with the previous poses, the cross covariance between the previous poses remain unchanged in this step
            if self.P_map.shape[0] > 3:
                for i in range(int(self.P_map.shape[0] / 3) ):
                    self.P_map[3*i:3*i+3,-3:] = self.P_map[3*i:3*i+3,-3:] @ J1.T

            if self.P_map.shape[1] > 3:
                for i in range(int(self.P_map.shape[0] / 3) ):
                    self.P_map[-3: , 3*i:3*i+3] = J1 @ self.P_map[-3: , 3*i:3*i+3] 
            
            
            # Reset flag
            self.left_wheel_received = False
            self.right_wheel_received = False

            # Publish odom

            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "world"
            odom.child_frame_id = "kobuki/base_footprint"

            odom.pose.pose.position.x = self.X[0]
            odom.pose.pose.position.y = self.X[1]

            odom.pose.pose.orientation.x = self.q[0]
            odom.pose.pose.orientation.y = self.q[1]
            odom.pose.pose.orientation.z = self.q[2]
            odom.pose.pose.orientation.w = self.q[3]

            
            P_list = self.P.tolist() # chaning the covariance from np.array to list for convenience
            

            # update the elements in odom.pose.covariance
            odom.pose.covariance[0] = np.sqrt(abs(P_list[0][0]))    # cov(x,x)
            odom.pose.covariance[7] = np.sqrt(abs(P_list[1][1]))    # cov(y,y)
            odom.pose.covariance[35] = np.sqrt(abs(P_list[2][2]))   # cov(yaw,yaw)

            odom.pose.covariance[1] = np.sqrt(abs(P_list[0][1]))    # cov(x,y)
            odom.pose.covariance[6] = np.sqrt(abs(P_list[1][0]))    # cov(y,x)

            odom.pose.covariance[5] = np.sqrt(abs(P_list[0][2]))   # cov(x,yaw)
            odom.pose.covariance[11] = np.sqrt(abs(P_list[1][2]))   # cov(y,yaw)
            odom.pose.covariance[30] = np.sqrt(abs(P_list[2][0]))   # cov(yaw,x)
            odom.pose.covariance[31] = np.sqrt(abs(P_list[2][1]))   # cov(yaw,y)
            
            
            odom.twist.twist.linear.x = self.v
            odom.twist.twist.angular.z = self.w

            self.odom_pub.publish(odom)

            
            self.tf_br.sendTransform((self.X[0], self.X[1], 0.0), self.q, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id)

    
    def imu_callback(self, msg):
        
        if not self.update_called:
            # Getting the euler yaw angle from the imu message
            euler = euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
            
            z = np.array([euler[2]]).reshape(1,1)
            h= self.Xk [-1,-1].reshape(1,1)
            
            # Linear Observation Jacobian
            H = np.zeros((1, len(self.Xk))).reshape(1,-1)
            H[0,-1] = 1
            V = np.eye(1)   

            K = self.P_map @ H.T @ np.linalg.inv( H @ self.P_map @ H.T + V @ self.imu_noise @ V.T)
            
            # updating the mean value of the state vector
            self.Xk = self.Xk + K @ (z - h)
            
            
            # updating the covariance  
            self.P_map = (np.eye(self.P_map.shape[0]) - K @ H ) @ self.P_map 
            
        
            self.P = self.P_map[-3:,-3:]
            self.X = self.Xk[-3:]
            
            

    def lidar_callback(self, msg):
    
        if (np.linalg.norm(self.last_scan_pose[:2] - self.X[:2]) > self.dis_thresh): # after the robot has moved some distance take new scan
            
            
            self.last_scan_pose = np.copy(self.X)
            laser_projector = LaserProjection()
            point_cloud_msg = laser_projector.projectLaser(msg)

            point_cloud = pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            points = []

            for point in point_cloud:
                points.append([float(point[0]), float(point[1]), float(point[2])])

            new_point_cloud = np.array(points).reshape(-1,3)
            
            
            # Access the timestamp of the message
            self.timestamp = msg.header.stamp.to_sec()
            rospy.loginfo("New scan received at time: {}".format(self.timestamp))
            
            # Calling the Update method where the correction step of EKF is done
            self.update_called = True
            self.update(new_point_cloud)

            
    """
        A function used to check possible overlapping scans.
        The function will return, the latest scan and at most 2 old scans that where taken near to the position from which the new scan was taken.
    """
            
                
    def check_overlap(self):
        overlaps = []
        if len(self.scans)>0:
            for i in range(len(self.scans)):
            
                if abs(self.timestamp - self.scan_time_stamp[i]) > 150 and (np.linalg.norm(self.Xk[3*i : 3*i + 2] - self.Xk[-3:-1]) <= self.overlap_thresh)and len(overlaps)<2: 
                    overlaps.append(i)
            
            overlaps.append(len(self.scans)-1)
        return overlaps
    
    """
        A function used to clone robot pose. 
    """
    
    def Add_New_Pose(self):
        # grow the state vector and the covariance matrix
        
        self.Xk = np.append (self.Xk, self.Xk[-3:], axis=0)
        self.Xk = np.reshape(self.Xk, (-1, 1))
        

        # Add one row and one column to the last row and column of the self.P_map
        self.P_map = np.pad(self.P_map, ((0,3), (0,3)), mode='constant')
        self.P_map[:,-3:] = self.P_map[:,-6:-3]
        self.P_map[-3:,:] = self.P_map[-6:-3,:]

        


    # Correction step for the Extended kalman filter is done here
    def update(self, data):   
        
        over_laps = self.check_overlap()
        overlap_scan_found = False
        
        if len(over_laps) > 0:
            
            for i in over_laps:

                # Observation model
                h = np.array ([[  -self.Xk[3*i]*np.cos(self.Xk[3*i+2]) - self.Xk[3*i+1] * np.sin(self.Xk[3*i+2]) + self.Xk[-3]* np.cos(self.Xk[3*i+2]) + self.Xk[-2]* np.sin(self.Xk[3*i+2]) ] , [ self.Xk[3*i]*np.sin(self.Xk[3*i+2]) - self.Xk[3*i+1] * np.cos(self.Xk[3*i+2]) - self.Xk[-3]* np.sin(self.Xk[3*i+2]) + self.Xk[-2] * np.cos(self.Xk[3*i+2]) ], [ wrap_angle(self.Xk[-1] - self.Xk[3*i+2] ) ] ]).reshape(3,1)
                
                
                # temp1 =  del h / del x, del h / del y, del h / del theta ---- > with the respect to the pose of the matching scan which was already in the state vector
                temp1 = np.array([ [-np.cos(self.Xk[3*i+2][0]), -np.sin(self.Xk[3*i+2][0]), self.Xk[3*i][0] * np.sin(self.Xk[3*i+2][0]) - self.Xk[3*i+1][0]* np.cos(self.Xk[3*i+2][0]) - self.Xk[-3][0]* np.sin(self.Xk[3*i+2][0]) + self.Xk[-2][0]* np.cos(self.Xk[3*i+2][0]) ], [ np.sin(self.Xk[3*i+2][0]), -np.cos(self.Xk[3*i+2][0]), self.Xk[3*i][0] * np.cos(self.Xk[3*i+2][0]) + self.Xk[3*i+1][0]* np.sin(self.Xk[3*i+2][0]) - self.Xk[-3][0]* np.cos(self.Xk[3*i+2][0]) - self.Xk[-2][0]* np.sin(self.Xk[3*i+2][0])], [0, 0, -1]  ])
                temp1 = temp1.reshape(3, 3)
                # temp2 =  del h / del x, del h / del y, del h / del theta ---- > with the respect to the pose from which the new scan was taken 
                temp2 = np.array([[np.cos(self.Xk[3*i+2][0]), np.sin(self.Xk[3*i+2][0]), 0], [-np.sin(self.Xk[3*i+2][0]), np.cos(self.Xk[3*i+2][0]), 0], [0, 0, 1]]).reshape(3,3)

                # Observation Jacobian
                self.H = np.zeros((3, len(self.Xk))).reshape(3,-1)

                self.H[:, 3*i:3*i+3] = temp1
                self.H[:, -3:] = temp2
                self.H.reshape(3,-1)

                self.V = np.eye(3)
               
                self.z = icp_register(data,self.scans[i], h)
                self.z = self.z.reshape(3,1)
                
            
            
                # UPDATING STEP OF KALMAN FILTER
              

                # Compatibility Test
                
                J_minus = np.array([[-np.cos(self.Xk[3*i+2][0]), -np.sin(self.Xk[3*i+2][0]), self.Xk[3*i][0]*np.sin(self.Xk[3*i+2][0])- self.Xk[3*i+1][0]*np.cos(self.Xk[3*i+2][0])], [np.sin(self.Xk[3*i+2][0]), -np.cos(self.Xk[3*i+2][0]), self.Xk[3*i][0]*np.cos(self.Xk[3*i+2][0])+ self.Xk[3*i+1][0]*np.sin(self.Xk[3*i+2][0])], [0, 0, -1 ]]).reshape(3,3)
                J1_plus = temp1
                J2_plus = temp2

                inverted_cov = J_minus @ self.P_map[3*i:3*i+3,3*i:3*i+3] @ J_minus.T
                S = J1_plus @ inverted_cov @ J1_plus.T + J2_plus @ self.P_map[-3:,-3:] @ J2_plus.T
                

                mahalanobis_dist = distance.mahalanobis(np.ravel(h), np.ravel(self.z), np.linalg.inv(S + self.R))
                
                
                print("Measurement compatible?  ", (mahalanobis_dist**2) <=self.critical_value)
                
                # Updating if measurement and expected measurement pass compatibility test
                if (mahalanobis_dist**2) <=self.critical_value:
                    # computing the kalman gain
                    
                    overlap_scan_found = True
                    K = self.P_map @ self.H.T @ np.linalg.inv( self.H @ self.P_map @ self.H.T + self.V @ self.R @ self.V.T)
                    
                    # updating the mean value of the state vector
                    self.Xk = self.Xk + K @ (self.z - h)
                    
        
                    
                    # updating the covariance  
                    self.P_map = (np.eye(self.P_map.shape[0]) - K @ self.H ) @ self.P_map # @ ( np.eye(self.P_map.shape[0]) - K @ self.H).T 
                    
                
            
            # for visualization purpose
            a = copy.deepcopy(self.Xk)

            # Create the header for the PointCloud2 message
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'  # Set the frame ID of the point cloud

            # Create the PointCloud2 message
            point_cloud_msg = pc2.create_cloud_xyz32(header, data)

            # Publishing the point clouds with  the transformation from dead reckoning robot state estimation
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()  # Set the timestamp
            transform.header.frame_id = 'world'  
            transform.child_frame_id = '/kobuki/base_footprint'  
            transform.transform.translation.x =  self.Xdeadreck[0]
            transform.transform.translation.y = self.Xdeadreck[1]
            transform.transform.translation.z = 0
            q = quaternion_from_euler(0, 0, self.Xdeadreck[2][0])
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]

            
            transformed_point_cloud = do_transform_cloud(point_cloud_msg, transform)
            
            self.deadReck_PC_pub.publish(transformed_point_cloud)

            # Publishing the point clouds with  the transformation from ground truth of the robot state
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()  # Set the timestamp
            transform.header.frame_id = 'world'  
            transform.child_frame_id = '/kobuki/base_footprint'  
            transform.transform.translation.x = self.XGroundTruth[0]
            transform.transform.translation.y = self.XGroundTruth[1]
            transform.transform.translation.z = 0
            q = quaternion_from_euler(0, 0, self.XGroundTruth[2])
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            transformed_point_cloud = do_transform_cloud(point_cloud_msg, transform)
            self.Grnd_truth_PC_pub.publish(transformed_point_cloud)

            # Publishing the point clouds with  the transformation from the pose based SLAM robot state estimation
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()  # Set the timestamp
            transform.header.frame_id = 'world'  
            transform.child_frame_id = '/kobuki/base_footprint'  
            transform.transform.translation.x = a[-3]
            transform.transform.translation.y = a[-2]
            transform.transform.translation.z =  0
            q = quaternion_from_euler(0, 0, a[-1])
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            transformed_point_cloud = do_transform_cloud(point_cloud_msg, transform)
            self.slam_PC_pub.publish(transformed_point_cloud)
            

            # for visualizing the evoluiton of the robot state uncertainity with time
            marker_array = MarkerArray()  # create a new MarkerArray message
            

            for m in range(int(a.shape[0] / 3)):
                
                # Create a new Modem marker message for robot pose display in rviz
                robot_pose = Marker()
                robot_pose.header.frame_id = "world"
                robot_pose.id= m
                robot_pose.header.stamp = rospy.Time.now()
            
                # Set the marker type and action
                robot_pose.type = Marker.SPHERE
                robot_pose.action = Marker.ADD

                # Set the color of the marker (RGBA) Blue
                robot_pose.color.r = 1.0
                robot_pose.color.g = 1.0
                robot_pose.color.b = 0.0
                robot_pose.color.a = 1.0
            
                robot_pose.scale.x = 10 * np.sqrt(self.P_map[3*m, 3*m])
                robot_pose.scale.y = 10 * np.sqrt(self.P_map[3*m+1, 3*m+1])
                robot_pose.scale.z = 0.01
                robot_pose.pose.orientation.w = 1.0
                robot_pose.pose.position = Point(x=a[3*m], y=a[3*m+1], z=0)  # unique position for each Marker message

                marker_array.markers.append(robot_pose)  # add Marker to MarkerArray
            
            self.pose_pub.publish(marker_array.markers)

            

        #  storing the scans taken by the robot
        if len(self.scans) < self.scan_num and overlap_scan_found or len(self.scans)==0:

            self.scans.append(data)
            self.Add_New_Pose()
            t = copy.deepcopy(self.timestamp)
            self.scan_time_stamp.append(t)

        # Updating the robot state
        self.P = self.P_map[-3:,-3:]
        self.X = self.Xk[-3:]

    
        # Publish odom

        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "world"
        odom.child_frame_id = "kobuki/base_footprint"

        odom.pose.pose.position.x = self.X[0]
        odom.pose.pose.position.y = self.X[1]

        self.q = quaternion_from_euler(0, 0, self.X[2][0])

        odom.pose.pose.orientation.x = self.q[0]
        odom.pose.pose.orientation.y = self.q[1]
        odom.pose.pose.orientation.z = self.q[2]
        odom.pose.pose.orientation.w = self.q[3]

        
        P_list = self.P.tolist() # changing the covariance from np.array to list for convenience
        

    

        # update the elements in odom.pose.covariance
        odom.pose.covariance[0] = np.sqrt(abs(P_list[0][0]))  # cov(x,x)
        odom.pose.covariance[7] = np.sqrt(abs(P_list[1][1]))   # cov(y,y)
        odom.pose.covariance[35] = np.sqrt(abs(P_list[2][2]))   # cov(yaw,yaw)

        odom.pose.covariance[1] = np.sqrt(abs(P_list[0][1]))    # cov(x,y)
        odom.pose.covariance[6] = np.sqrt(abs(P_list[1][0]))    # cov(y,x)

        odom.pose.covariance[5] = np.sqrt(abs(P_list[0][2]))   # cov(x,yaw)
        odom.pose.covariance[11] = np.sqrt(abs(P_list[1][2]))   # cov(y,yaw)
        odom.pose.covariance[30] = np.sqrt(abs(P_list[2][0]))   # cov(yaw,x)
        odom.pose.covariance[31] = np.sqrt(abs(P_list[2][1]))   # cov(yaw,y)
        
        
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

        self.update_called= False

        self.tf_br.sendTransform((self.X[0], self.X[1], 0.0), self.q, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id)




if __name__ == '__main__':

    rospy.init_node("Slam_node")

    robot = DifferentialDrive()

    

    rospy.spin()

    





