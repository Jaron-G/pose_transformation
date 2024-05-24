#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from std_msgs.msg import Int32
import math
from scipy.spatial.transform import Rotation as R
import transforms3d as tfs

def rotation_matrix_to_euler_angles(R):
    assert R.shape == (3, 3)
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])  # roll, pitch, yaw

def r_t_to_homogeneous_matrix(R, T):
    R1 = np.vstack([R, np.array([0, 0, 0])])
    T1 = np.vstack([T, np.array([1])])
    HomoMtr = np.hstack([R1, T1])
    return HomoMtr

def homogeneous_matrix_to_r_t(HomoMtr):
    R = HomoMtr[:3, :3]
    T = HomoMtr[:3, 3]
    return R, T

def is_rotated_matrix(R):
    Rt = np.transpose(R[:3, :3])
    shouldBeIdentity = np.dot(Rt, R[:3, :3])
    I = np.eye(3)
    return np.linalg.norm(I - shouldBeIdentity) < 1e-6

def inverse_matrix(matrix):
    if np.linalg.det(matrix) != 0:
        inverse_matrix = np.linalg.inv(matrix)
    else:
        print("The matrix is not invertible.")
        return inverse_matrix

def rotation_matrix_to_axis_angle(R):
    assert R.shape == (3, 3)

    angle = math.acos((np.trace(R) - 1) / 2)
    r = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])
    axis = r / (2 * math.sin(angle))

    # Scale axis by the angle to get the vector with magnitude equal to the angle
    axis_with_angle = axis * angle

    return axis_with_angle

def generate_pose(homo_matrix):
        tempR, tempT = homogeneous_matrix_to_r_t(homo_matrix)
        euler_angles = [0,0,0]
        euler_angles[2],euler_angles[1],euler_angles[0] = tfs.euler.mat2euler(tempR, axes='szyx')
        axis_angles = rotation_matrix_to_axis_angle(tempR)
        pose_axis = np.hstack([tempT.flatten(), axis_angles])
        print("The pose(axis angles) is:", pose_axis)
        pose_euler = np.hstack([tempT.flatten(), euler_angles])
        return pose_euler, tempR

def PoseEst(matched_item, trans_matrix):
        if matched_item == 'E3u':
            t_vec = np.array([[34], [200], [33]])
            t_vec_up = np.array([[34], [397], [33]])
            r_vec = np.array([math.pi/2, 0, 0], dtype=np.float64)
        elif matched_item == 'A1':
            t_vec = np.array([[37], [37], [204]])
            t_vec_up = np.array([[37], [37], [394]])
            r_vec = np.array([math.pi, 0, 0], dtype=np.float64)
        elif matched_item == 'A2':
            t_vec = np.array([[34], [34], [204]])
            t_vec_up = np.array([[34], [34], [394]])
            r_vec = np.array([math.pi, 0, 0], dtype=np.float64)
        elif matched_item == 'A6':
            t_vec = np.array([[34], [34], [204]])
            t_vec_up = np.array([[34], [34], [394]])
            r_vec = np.array([math.pi,  0, 0], dtype=np.float64)
        elif matched_item == 'B4':
            t_vec = np.array([[34], [34], [204]])
            t_vec_up = np.array([[34], [34], [394]])
            r_vec = np.array([math.pi,  0, 0], dtype=np.float64)
        elif matched_item == 'D2':
            t_vec = np.array([[34], [34], [204]])
            t_vec_up = np.array([[34], [34], [394]])
            r_vec = np.array([math.pi,  0, 0], dtype=np.float64)
        elif matched_item == 'D4':
            t_vec = np.array([[204], [34], [34]])
            t_vec_up = np.array([[394], [34], [34]])
            r_vec = np.array([0,  -math.pi/2, math.pi/2], dtype=np.float64)
        elif matched_item == 'E0s':
            t_vec = np.array([[0], [0], [170]])
            t_vec_up = np.array([[0], [0], [170]])
            r_vec = np.array([0, 0, 0], dtype=np.float64)
        elif matched_item == 'E1u':
            t_vec = np.array([[34], [37], [203]])
            t_vec_up = np.array([[34], [37], [393]])
            r_vec = np.array([math.pi, 0, math.pi], dtype=np.float64)
        elif matched_item == 'C0':
            t_vec = np.array([[35], [35], [-135]])
            t_vec_up = np.array([[35], [35], [-325]])
            r_vec = np.array([0, 0, 0], dtype=np.float64)
        elif matched_item == 'D1':
            t_vec = np.array([[35], [35], [195]])
            t_vec_up = np.array([[35], [35], [395]])
            r_vec = np.array([math.pi, 0, 0], dtype=np.float64)
        elif matched_item == 'A3':
            t_vec = np.array([[34], [204], [34]])
            t_vec_up = np.array([[34], [394], [34]])
            r_vec = np.array([math.pi/2, 0, 0], dtype=np.float64)
        elif matched_item == 'pipe':
            t_vec = np.array([[30], [10], [-160]])
            t_vec_up = np.array([[30], [10], [-280]])
            r_vec = np.array([0, 0, 0], dtype=np.float64)
        elif matched_item == 'F1':
            t_vec = np.array([[35], [37.5], [-136.5]])
            t_vec_up = np.array([[35], [37.5], [-326.5]])
            r_vec = np.array([0, 0, -math.pi], dtype=np.float64)
        else:
            print("The matched number is", matched_item)
        pR_matrix = tfs.euler.euler2mat(r_vec[2],r_vec[1],r_vec[0],'szyx')
        # print("the designed rotation matrix is:", pR_matrix)
        pose_matrix = r_t_to_homogeneous_matrix(pR_matrix, t_vec)
        # print("the pose matrix is:", pose_matrix)
        up_pose_matrix = r_t_to_homogeneous_matrix(pR_matrix, t_vec_up)

        pose_trans_matrix = np.matmul(trans_matrix, pose_matrix)
        # print("The result matrix is:", pose_trans_matrix)
        up_pose_trans_matrix = np.matmul(trans_matrix, up_pose_matrix)
        grasp_pose_euler, tempR= generate_pose(pose_trans_matrix)
        # print("The grasp pose is", grasp_pose_euler)
        up_pose_euler, tempR = generate_pose(up_pose_trans_matrix)
        # print("The pre grasp pose is", up_pose_euler)
        return tempR, grasp_pose_euler, up_pose_euler

if __name__ == "__main__":
    rospy.init_node('pose_tranformation_node')
    final_pose = np.array([[7.07361985e-01, 7.06851485e-01,  4.21206373e-07,  240],
    [-7.06851485e-01,  7.07361985e-01,  2.34742508e-05, -700],
    [1.62948636e-05, -1.69025230e-05,  1,  1010],
    [0,  0,  0,  1]])
    matched_model = 'pipe'

    print("The final_pose is:", final_pose)
    
    trans_matrix = np.loadtxt('/catkin_ws/src/pose_transformation_pkg/out/matrix.txt')
    final_trans_pose = np.matmul(trans_matrix, final_pose)
    print("The final_trans_pose is:", final_trans_pose)
    
    rotation_matrix, pose_g, pose_up = PoseEst(matched_model, final_trans_pose)
    print("Pose_grasp:", pose_g)
