import rospy
import math
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R
import numpy as np
import pyrealsense2 as rs
import rospy
import cv2
import cv2.aruco as aruco
from transforms3d import *
from scipy.spatial.transform import Rotation as R

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def aruco_display(corners, ids, rejected, image):

    if len(corners) > 0:

        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(markerID))
        else:
            print("[Inference] No ArUco markers detected")

    return image

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #print(len(rejected))
    if len(corners) > 0:
        for i in range(0, len(ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], .0508, matrix_coefficients,
                                                                           distortion_coefficients)

            cv2.aruco.drawDetectedMarkers(frame, corners)
        return rvec, tvec
    else:
        print("No Aruco markers detected")
        rvec = [[0.0], [0.0], [0.0]]
        tvec = [[0.0], [0.0], [0.0]]
        return rvec, tvec

def get_aruco_t_camera(color_image, intrinsic, aruco_type):
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
    aruco_params = cv2.aruco.DetectorParameters_create()

    color_image = cv2.copyMakeBorder(color_image, 0, 0, 0, 40, cv2.BORDER_CONSTANT)
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)
    if ids is not None:
        for i in range(0, len(ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], .1, intrinsic, .01)

        marker_image = cv2.aruco.drawDetectedMarkers(gray_image, corners)
        marker_image = cv2.drawFrameAxes(marker_image, intrinsic, .01, rvec, tvec, .1)
        cv2.imshow('marker image', marker_image)
        # print(rvec)
        aruco_t_camera = np.concatenate((tvec, rvec), -1)
        aruco_t_camera = np.reshape(aruco_t_camera, -1)
        return aruco_t_camera
    else:
        print('No aruco tag detected')
        return np.zeros(6)

def get_intrinsics(stream_profile:rs.pyrealsense2.stream_profile):
    prof = rs.video_stream_profile(stream_profile)
    intrinsics = prof.get_intrinsics()
    properties = ['coeffs', 'fx', 'fy', 'ppx', 'ppy', 'width', 'height']
    return {p:getattr(intrinsics, p) for p in properties if hasattr(intrinsics, p)}

def get_K(intrinsics):
    '''Returns the intrinsic matrix of the RGB sensor'''
    k = np.eye(3)
    k [0, 0] = intrinsics["fx"]
    k [1, 1] = intrinsics["fy"]
    k [0, 2] = intrinsics["ppx"]
    k [1, 2] = intrinsics["ppy"]
    return k 

def trans_eval(ee_T_obj, aruco_gt):
    listener = tf.TransformListener()
    trans = []
    while not trans:
        try:
            (trans,rot) = listener.lookupTransform('world', 'robotiq_85_base_link', rospy.Time(0))
            rot = tf.transformations.euler_from_quaternion(rot)
            world_T_ee = np.zeros((4, 4))
            world_T_ee[:3, :3] = R.from_euler('xyz', rot, degrees=False).as_matrix()
            world_T_ee[:3, 3] = trans
            world_T_ee[3, 3] = 1
            
            obj_T_world = np.matmul(world_T_ee, ee_T_obj)
            # print(obj_T_world)
            # convert to 6D pose    
            obj_eul = R.from_matrix(obj_T_world[:3, :3]).as_euler('xyz', degrees=False)
            obj_T_world_eval = [obj_T_world[0, 3], obj_T_world[1, 3], obj_T_world[2, 3], obj_eul[0], obj_eul[1], obj_eul[2]]

            rmse = np.zeros(6)
            # rmse[0] = np.array(obj_T_world_eval[0]) - np.array(aruco_gt[0]) 
            # rmse[1] = np.array(obj_T_world_eval[1]) - np.array(aruco_gt[1])
            # rmse[2] = np.array(obj_T_world_eval[2]) - np.array(aruco_gt[2]) 
            # rmse[3] = np.array(obj_T_world_eval[3]) - np.array(aruco_gt[3]) 
            # rmse[4] = np.array(obj_T_world_eval[4]) - np.array(aruco_gt[4]) 
            # rmse[5] = np.array(obj_T_world_eval[5]) - np.array(aruco_gt[5]) 



            rmse[0] = np.sqrt(np.mean(np.abs((np.array(obj_T_world_eval[0]) - np.array(aruco_gt[0])))))
            rmse[1] = np.sqrt(np.mean(np.abs((np.array(obj_T_world_eval[1]) - np.array(aruco_gt[1])))))
            rmse[2] = np.sqrt(np.mean(np.abs((np.array(obj_T_world_eval[2]) - np.array(aruco_gt[2])))))
            rmse[3] = np.sqrt(np.mean(np.abs((np.array(obj_T_world_eval[3]) - np.array(aruco_gt[3])))))
            rmse[4] = np.sqrt(np.mean(np.abs((np.array(obj_T_world_eval[4]) - np.array(aruco_gt[4])))))
            rmse[5] = np.sqrt(np.mean(np.abs((np.array(obj_T_world_eval[5]) - np.array(aruco_gt[5])))))
            # print(rmse)
            return rmse, obj_T_world

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
    
    # print(ee_T_world)
