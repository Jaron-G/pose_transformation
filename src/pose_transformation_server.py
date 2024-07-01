#!/usr/bin/env python3
from __future__ import print_function
from pose_transformation.srv import PoseTransform,PoseTransformResponse #注意是功能包名.srv
import rospy
from std_msgs.msg import Int32

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import transforms3d as tfs

from  PoseTransformation import PoseTrans

import argparse  
parser = argparse.ArgumentParser(description='Set hand eye matrix path')
parser.add_argument('hand_eye_matrix_path', type=str, help='the path of hand_eye_matrix')
args = parser.parse_args(rospy.myargv()[1:]) # 获取排除ros自身启动节点外所有参数：__name:=xxx __log:=/root/.ros/log/xxx.log


def handle_pose_transform(req):
    final_pose = np.array(req.final_pose).reshape(4,4)
    matched_model = req.matched_matched_model
    rotation_matrix, pose_g, pose_up = PoseTrans(matched_model, final_pose, args.hand_eye_matrix_path)
    rotation_matrix, grasp_pose, up_pose  = rotation_matrix.reshape(-1).tolist(),pose_g.tolist(),pose_up.tolist()
    return PoseTransformResponse(rotation_matrix, grasp_pose, up_pose)

if __name__ == "__main__":
    rospy.init_node('pose_transformation_server')
    server = rospy.Service('pose_transformation', PoseTransform, handle_pose_transform)
    rospy.loginfo("Ready to pose transformation.")
    rospy.spin()


