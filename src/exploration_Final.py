#!/usr/bin/python3

import rospy
import numpy as np
import cv2
import copy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseArray

'''
Function extracts information from the gridmap.
'''

def get_gridmap(gridmap):
    global origin, resolution, env
    
    env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
    origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
    resolution = gridmap.info.resolution 
    getfrontier(gridmap)

""" a funtion that is used to convert from world coordinate to map (row, column) coordinate
 input:
   p: list representing the x and y coordinate of a position in the world
   
   output:
   Numpy: [row, column] """
def __position_to_map__(p):
	
	# convert world position to map coordinates. If position outside map return `None`
	map_coor = (np.array([p[0], p[1]]) - origin) / resolution
        
	# convert to integer
	map = np.round(map_coor).astype(int)
 
	# Check if the cell position is within the map boundaries
	if np.any(map < 0) or np.any(map >= np.shape(env)):
		return None
	else:
		return map


""" a funtion that is used to compute if a position (x,y) is in collision or not
 input:
   x: numpy array representing the x and y coordinate of a position in the world
   
   output:
   boolean: True if the position is in collision, False otherwise """

def IsInCollision(x):
    
    if len(x) == 0:
        return True  # Empty input, consider it as collision
    
    is_unknown_valid = True  # Flag to determine if unknown cells are considered valid
    
    # Create an array `s` of shape (1, 2) and initialize it with zeros
    s = np.zeros(2, dtype=np.float32).reshape(1, 2)
    
    distance = 0.16  # Distance threshold for collision
    
    s[0, 0] = x[0]  # Assign the x-coordinate of `x` to `s`
    s[0, 1] = x[1]  # Assign the y-coordinate of `x` to `s`
    
    # Convert the coordinates of `s` to corresponding map position
    point2 = __position_to_map__([s[0, 0], s[0, 1]])
    
    # Check if `point2` is None or if the value at `point2` in `env` is greater than 50
    if point2 is None or env[point2[0], point2[1]] > 30:
        return True  # Point is out of bounds or exceeds collision threshold
    
    x, y = point2[0], point2[1]  # Assign the x and y coordinates from `point2`
    
    dist = int(math.ceil(distance / resolution))  # Calculate the distance threshold in map cells
    
    # Iterate over the neighboring cells within the distance threshold
    for i in range(-dist, dist + 1):
        for j in range(-dist, dist + 1):
            x_cor, y_cor = x + i, y + j  # Calculate the x and y coordinates of the current neighboring cell
            
            # Check if the neighboring cell is within the bounds of the environment
            if x_cor >= 0 and x_cor < env.shape[0] and y_cor >= 0 and y_cor < env.shape[1]:
                # Check if the value at the neighboring cell is above a collision threshold
                if env[x_cor, y_cor] >= 30:
                    return True  # Neighbor cell exceeds collision threshold
                # Check if the value at the neighboring cell is -1 (unknown) and unknown cells are not considered valid
                if env[x_cor, y_cor] == -1 and not is_unknown_valid:
                    return True  # Neighbor cell is unknown and considered invalid
            else:
                # The neighboring cell is out of bounds and unknown cells are not considered valid
                if not is_unknown_valid:
                    return True  # Neighbor cell is out of bounds and considered invalid

    return False  # Point is not in collision with the environment 
        
""" 
    getfrontier processes a gridmap data and extracts frontiers, which are areas between explored free cells and unexplored regions. 
    It uses OpenCV functions for image processing, contour detection, and centroid calculation. The function then publishes 
    all the detected frontiers and the best frontier as a goal point to be explored if a valid frontier is found.
"""

def getfrontier(mapData):
    
    rospy.loginfo_once("---- 'getfrontier' node is loaded ----")
    

    # Extract necessary information from mapData
    data = mapData.data
    width = mapData.info.width
    height = mapData.info.height
    resolution = mapData.info.resolution
    startx = mapData.info.origin.position.x
    starty = mapData.info.origin.position.y 

    # Create an image matrix to represent the map
    img = np.zeros((height, width, 1), np.uint8)

    # Populate the image matrix with values based on map data
    for i in range(0, height):
        for j in range(0, width):
            if data[i * width + j] == 100:
                img[i, j] = 0   # occupied cell
            elif data[i * width + j] == 0:
                img[i, j] = 100  # free cell
            elif data[i * width + j] == -1:
                img[i, j] = 50  # unexplored / unknown cell
    
    # Threshold the image to obtain a binary map (occupied cells: 255, free and unknown cells: 0)
    map_bin = cv2.inRange(img, 0, 1)
    
    # Apply the Canny edge detection algorithm to detect edges in the map
    edges = cv2.Canny(img, 100, 200)

    # Find contours in the binary map
    contours,_ = cv2.findContours(map_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(map_bin, contours, -1, (255,255,255), 5)

    # Invert the binary map (occupied cells: 0, free and unknown cells: 255)
    map_bin = cv2.bitwise_not(map_bin)

    # Perform a bitwise AND operation between the inverted map and the edges to obtain the frontier
    res = cv2.bitwise_and(map_bin, edges)

    # Create a deep copy of the frontier
    frontier = copy.deepcopy(res)

    # Find contours in the frontier
    contours, _ = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frontier, contours, -1, (255,255,255), 2)
    contours, _ = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frontier, contours, -1, (255,255,255), 1)

    # Initialize a list to store all points of the frontier
    all_pts = []
    
    if len(contours) > 0:
        best_frontier = []
        max_len = 0
        
        # Iterate over the contours to process each one
        for i in range(len(contours)):

            # Calculate the length of the contour
            length = cv2.arcLength(contours[i], True)

            # Convert contour points to a 2d numpy array
            arr = np.squeeze(contours[i], axis=1)

            # Calculate the centroid of the contour
            cx = np.mean(arr[:, 0]) 
            cy = np.mean(arr[:, 1]) 

            # Convert the centroid to its corresponding position in the global map
            xr = cx * resolution + startx
            yr = cy * resolution + starty

            # Create a point with the calculated position
            pt = [np.array([xr, yr])]
        

            # Check if the point is not in collision, if not in collision store it as a frontier
            if not IsInCollision(pt[0]):
                if len(all_pts) > 0:
                    all_pts = np.vstack([all_pts, pt])
                else:
                    all_pts = pt

                # Update the best frontier if the length is greater than the previous maximum, the frontier is also checked if it is within the search area
                if (length > max_len) and 0.6 < pt[0][0] < 4.50 and 0.5 < pt[0][1] < 2.7:
                    best_frontier = pt[0]
                    max_len = length

    # Check if exploration is completed 
    if len(best_frontier) == 0:
        print("Exploration Completed!")
    
    # Publish as a goal if a valid best frontier is found
    if len(best_frontier) > 0:
        goal = best_frontier

        veiw_pt_msg = PoseStamped()
        veiw_pt_msg.pose.position.x = goal[0]
        veiw_pt_msg.pose.position.y = goal[1]
        veiw_pt_msg.pose.position.z = 0.0
        veiw_pt_msg.pose.orientation.x = 0.0
        veiw_pt_msg.pose.orientation.y = 0.0
        veiw_pt_msg.pose.orientation.z = 0.0
        veiw_pt_msg.pose.orientation.w = 1.0
        veiw_pt_msg.header.frame_id = 'world'  # Set the frame ID
        veiw_pt_msg.header.stamp = rospy.Time.now()  # Set the timestamp
        exploration_goal_pub.publish(veiw_pt_msg)
        goal = best_frontier
        best_frontier = []
     
    # Publish an unreachable goal if no frontier is found
    elif len(best_frontier) == 0:
        veiw_pt_msg = PoseStamped()
        veiw_pt_msg.pose.position.x = 123456.0
        veiw_pt_msg.pose.position.y = 123456.0
        veiw_pt_msg.pose.position.z = 0.0
        veiw_pt_msg.pose.orientation.x = 0.0
        veiw_pt_msg.pose.orientation.y = 0.0
        veiw_pt_msg.pose.orientation.z = 0.0
        veiw_pt_msg.pose.orientation.w = 1.0
        veiw_pt_msg.header.frame_id = 'world'  # Set the frame ID
        veiw_pt_msg.header.stamp = rospy.Time.now()  # Set the timestamp
        exploration_goal_pub.publish(veiw_pt_msg)
        goal = best_frontier
        best_frontier = []
    
    # Publish all points of the frontier
    if len(all_pts) != 0:
        msg = PoseArray()
        msg.header.frame_id = 'world'  # Set the frame ID
        msg.header.stamp = rospy.Time.now()  # Set the timestamp

        for i in range(len(all_pts)):
            veiw_pt_msg = Pose()
            veiw_pt_msg.position.x = all_pts[i][0]
            veiw_pt_msg.position.y = all_pts[i][1]
            veiw_pt_msg.position.z = 0.0
            veiw_pt_msg.orientation.x = 0.0
            veiw_pt_msg.orientation.y = 0.0
            veiw_pt_msg.orientation.z = 0.0
            veiw_pt_msg.orientation.w = 1.0

            msg.poses.append(veiw_pt_msg)

        Frontier_pub.publish(msg)

    


if __name__ == '__main__':
    
    rospy.init_node("exploration_node") 
    rospy.Subscriber('/projected_map', OccupancyGrid, get_gridmap)
    # create a publisher for the best frontier
    exploration_goal_pub = rospy.Publisher('/explration_view_point', PoseStamped, queue_size=1)
    # create a publisher for all frontiers
    Frontier_pub = rospy.Publisher('/all_frontiers', PoseArray, queue_size=1)
    
    

    rospy.spin()