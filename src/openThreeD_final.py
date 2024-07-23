import open3d as o3d
import numpy as np
import math
from wrap_angle_final import *

def icp_register(scan1,scan2, T_initial):
    # Read point clouds 
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(scan1)

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(scan2)


    theta = T_initial[-1][0]  # in rad
    
    T = np.array([[np.cos(theta), -np.sin(theta), 0, T_initial[0][0]],
                [np.sin(theta), np.cos(theta), 0, T_initial[1][0]],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

    icp = o3d.registration.registration_icp(
        source=pcd1, 
        target=pcd2, 
        max_correspondence_distance=0.9, 
        init=T,
        estimation_method=o3d.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=500000))

    # Get the transformation matrix
    transformation = icp.transformation

    

   
    angle = math.atan2(transformation[1,0], transformation[0,0])
    angle = wrap_angle(angle)

    return np.array([transformation[0,-1], transformation[1,-1], angle])
