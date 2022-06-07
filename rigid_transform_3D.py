#!/usr/bin/python

import numpy as np
from scipy.spatial.transform import Rotation as R

# Input: expects 3xN matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B

    return R, t
    
# ~ cup_world = np.array([
    # ~ [0.7996, 0.29377, 0.02766, 1],
    # ~ [0.83942, 0.29035, 0.02672, 1],
    # ~ [0.78332, 0.32967, 0.02806, 1],
    # ~ [0.80459, 0.3626,  0.02982, 1],
    # ~ [0.86508,0.32381, 0.06080, 1]
    # ~ ])

# ~ cup_self = np.array([
    # ~ [-0.02596, -0.03429, 0.01282, 1],
    # ~ [0.01252, -0.03313, 0.01207, 1],
    # ~ [-0.04602, -0.002269, 0.01301, 1],
    # ~ [-0.02815, 0.03155, 0.01215, 1],
    # ~ [0.0359, 0.00258, 0.04238,1]
    # ~ ])

# ~ n_lines = 6
# ~ marker_measured = np.concatenate([np.loadtxt("points_cup.txt")[:n_lines],np.ones((n_lines,1))],-1)
# ~ marker_self = np.array([[0.03694,-0.00903,0.03876, 1],
					   # ~ [0.012909, -0.032898, 0.011997,1],
					   # ~ [-0.008129, -0.03392, -0.034582,1],
					   # ~ [-0.025245, -0.034687, 0.01265,1],
					   # ~ [0.035855, 0.001515, 0.042417,1],
					   # ~ [-0.03671, -0.018697, -0.033915,1]]
            # ~ )



marker_measured = np.concatenate([np.loadtxt("save_points_pointer_tip_np.txt"),np.ones((12,1))],-1)
marker_self = np.array([[0,0,0,1000],
                      [0,175,0,1000],
                      [175,175,0,1000],
                      [175,0,0,1000],
                      [35,35,0,1000],
                      [35,140,0,1000],
                      [140,140,0,1000],
                      [140,35,0,1000],
                      [70,70,0,1000],
                      [70,105,0,1000],
                      [105,105,0,1000],
                      [105,70,0,1000]])/1000

# ~ aruco_self = np.array([
    # ~ [175, 0, 0, 1],
    # ~ [175, 175, 0, 1],
    # ~ [0 , 0, 0, 1],
    # ~ [0, 175, 0 , 1]
# ~ ])
# ~ aruco_world = np.array([[360.01, 60.17, -30.78, 1],
            # ~ [359.97, 236.22, -29.18, 1],
            # ~ [533.88, 61.65, -31.35, 1],
            # ~ [533.69, 236.43,-29.23, 1]])

# ~ R, t = rigid_transform_3D(cup_self[:, :3].T, cup_world[:, :3].T)
# ~ print(R, t)

# check with aruco board
# R1, t1 = rigid_transform_3D(aruco_self[:, :3].T, aruco_world[:, :3].T)
# print(R1, t1)

Rot, t = rigid_transform_3D(marker_self[:, :3].T, marker_measured[:, :3].T)


T = np.eye(4)
T[:3, :3] = Rot
T[:3, 3] = t.T

print("as matrix")
print(T)
print("as quaternion with vector")
print(R.from_matrix(Rot).as_quat())
print(t.T)

np.savetxt("gt_checker_pose.txt",T)
# T1 = np.eye(4)
# T1[:3, :3] = R1
# T1[:3, 3] = t1.T

for idx,b in enumerate(marker_self):
    a = np.matmul(T, b)
    # print(a)
    print((a - marker_measured[idx])*1000,"error in mm")
