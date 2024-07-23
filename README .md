# Autonomous Exploration

The aim of this project is to enable a robot to autonomously explore previously unexplored areas and manipulate objects with its arm using a task priority algorithm.

## Members of the goup:

This project has been carried out by:

* Huy Nguyen
* Million Angesom
* Mohsin Kabir

## Prerequisites:
* ROS 
* autonomous_task_behaviorTree_final.py
* rrtstar_dwa_Final.py
* laser_scan_to_point_cloud_node.py
* HOL_Final.py
* exploration_Final.py
* openThreeD_final.py
* wrap_angle_final.py
* stonefish
* Opencv(cv2)
* Open3d library
## Usage:

To execute the project file, first open a command prompt and run the following commands:


```roslaunch Hands_on_Planning kobuki_basic.launch```

This command will launch the stonefish and rviz for the simulation.

In a new terminal, navigate to the `src` directory containing the Python script and run the following commands to execute the code:

```python3 HOL_Final.py```

This command will launch the localisation node. Then in other terminal, run this command to execute the exploration implementation.

```python3 exploration_Final.py```

Next, run the planning program to compute the path via this command:

```python3 rrtstar_dwa_Final.py```

Finally, by using the behavior tree the autonomous task is implemented: 

```python3 autonomous_task_behaviorTree_final.py```

The script will start automaticlly controlling the kobuki robot to explore in the environment. A launch file could also be used to run all the nodes at the same time.

## Functionality:
The code performs the following tasks:
1. *autonomous_task_behaviorTree_final.py*  The behavior tree consists of these behaviors:
    * Explore: Represents the exploration task. It listens to the /exploration_view_point topic for pose updates and updates the blackboard with the current location. It returns SUCCESS when the best view position is written to the blackboard and FAILURE when exploration is completed.

    * Perceive: Represents the perception task. It listens to the /perceived_point topic for pose updates and updates the blackboard with the current location. It returns SUCCESS when an object is detected and FAILURE otherwise.

    * Move_to: Represents the motion planner. It reads the location from the blackboard and publishes it to the /move_base_simple/goal topic. It returns SUCCESS when the feedback from planning node,  /planning_feedback topic, is "success," FAILURE when the feedback is "failure," and RUNNING otherwise.

    * Pick_And_Place_1: Represents the picking task. It reads the location from the blackboard and publishes it to the /end_effector_pose topic. It returns SUCCESS when the feedback from the /end_effector_pose_reached topic is True, FAILURE when the feedback is False, and RUNNING otherwise.

    * Pick_And_Place_2: Represents the placing task. It functions similarly to Pick_And_Place_1.
    

2. *rrtstar_dwa_Final.py* 
    * Tree Class: The Tree class represents the tree structure used by the RRT* algorithm:
    - __init__(self, init_node, goal_node): Initializes the tree with an initial node and a goal node.
    - RandomState(self, min_p, max_p): Generates a random state within the given bounds.
    - add_vertex(self, loc): Adds a new vertex to the tree if it does not already exist.
    - add_edges(self, new_idx, idx, distance): Adds edges between vertices in the tree.
    - rewire_edge(self, neighbor_idx, new_parent_idx): Rewires edges in the tree to improve connectivity.
    - get_path(self, goal_node): Returns the path from the initial node to the goal node.
    * RRTstar Class: The RRTstar class implements the RRT* algorithm and handles the overall path planning process.
    - __init__(self): Initializes the RRT* planner and sets up the required variables and subscribers.
    - get_odom(self, odom): Callback function to get the current robot pose from the odometry data. It is provided by localisation node (HOL_Final.py).
    - controller(self, event): Callback function for the controller that handles the movement of the robot towards the goal.
    - get_current_pose(self): Returns the current robot pose.
    - get_current_pose_robot(self): Returns the current robot pose in the form of (x, y, theta).
    - get_gridmap(self, gridmap): Callback function to get the grid map data.
    - get_goal(self, msg): Callback function to get the goal position from the exploration.
    - cell2world(self, map_coor): Converts cell position to world coordinates.
    - __position_to_map__(self, p): Converts world position to map coordinates.
    - IsInCollision(self, x, dist): Checks if a given state is in collision with obstacles.
    - euclidean(self, p1, p2): Calculates the Euclidean distance between two points.
    - NearestState(self, rand_node, T): Returns the nearest vertex, index, and minimum distance to the random node.
    - find_near_neighbors(self, new_node, gamma, T): Returns the near neighbors within a given radius.
    - NewState(self, near_node, rand_node, delta, min_dist): Generates a new vertex in the direction of the line connecting near and random nodes.
    - generateRRT_star(self, delta, gamma, near_dist, iterations): Generates an RRT* tree by iteratively expanding towards the goal.
    - publish_path(self): Publishes the planned path as a list of waypoints.
    - run(self): Runs the RRT* planner and the robot controller.

3. *laser_scan_to_point_cloud_node.py* 
    * This code converts LaserScan messages from a LiDAR sensor to PointCloud2 messages in ROS

4. *HOL_Final.py* estimates the position of the robot based on the slam method. 

5. *exploration_Final.py* includes several classes and functions related to exploration.
    * Receive occupancy grid map data and extracts frontiers.
    * Utilize OpenCV functions for image processing, contour detection, and centroid calculation.
    * The detected best frontier is published as a goal point to be explored.
    * The code performs the following functions:
    - get_gridmap(gridmap): Extracts information from the gridmap data, such as the environment grid, origin, and resolution.
    - __position_to_map__(p): Converts a world coordinate [x, y] to map (row, column) coordinates.
    - IsInCollision(x): Checks if a position [x, y] is in collision with the environment. It considers positions in collision if they are outside the map boundaries or exceed a collision threshold.
    - getfrontier(mapData): Processes the gridmap data and extracts frontiers. It uses OpenCV functions for image processing, contour detection, and centroid calculation. The detected best frontier is published as a goal point to be explored if a valid frontier is found.

6. *openThreeD_final.py* implements the ICP algorithm for point cloud registration using the Open3D library. The ICP algorithm aims to align two point clouds by finding the optimal rigid transformation that minimizes the distance between corresponding scans.

7. *wrap_angle_final.py* wraps an angle ang between -π and π.

## VIDEO DEMO:
1. [RRT* Visualisation](https://youtu.be/fZltKa4dIb4)
2. [Exploration Algorithm](https://youtu.be/qE3l-_JUnSA)
3. [Autonomous Exploration Real Robot](https://youtu.be/YqAs7LerJx0)

## Paper
The detailed explanation of the project can be accessed [here](https://drive.google.com/file/d/1c_-AcfYp6NJsLPngxOh2b0vQFReeB8I_/view).