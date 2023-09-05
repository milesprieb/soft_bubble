# Imports
import numpy as np
import pyrealsense2 as rs
import rospy
import struct
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler
from transforms3d import *
import tf
import cv2

# ROS msgs & services
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from sensor_msgs.msg import *
from sensor_msgs import point_cloud2
from visualization_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

def image_to_msg(image, request_header, image_condition):
    msg = Image()
    msg.height = image.shape[0]
    msg.width = image.shape[1]
    np_image = np.array(image)
    if np.issubdtype(np_image.dtype, np.floating):
        if np_image.max() <= 1:
            np_image *= 255
        else:
            raise RuntimeError("Ambiguous image values...")
    elif np_image.max() <= 1:
        np_image *= 255
    msg.header.frame_id = request_header.frame_id
    msg.header.stamp = request_header.stamp

    if len(image.shape) == 2:
        if image.dtype == np.uint8:
            msg.is_bigendian = True
            msg.encoding = 'mono8'
            msg.step = msg.width
            np_image = np_image.astype(np.uint8)
        elif image.dtype == np.uint16:
            msg.is_bigendian = False
            msg.encoding = '16UC1'
            msg.step = 2 * msg.width
            np_image = np_image.astype(np.uint16)
    elif len(image.shape) == 3:
        if image.shape[-1] == 3:
            msg.is_bigendian = True
            msg.encoding = 'rgb8'
            msg.step = 3 * msg.width
            np_image = np_image.astype(np.uint8)
        if image.shape[-1] == 4:
            msg.is_bigendian = True
            msg.encoding = 'rgba8'
            msg.step = 4 * msg.width
            np_image = np_image.astype(np.uint8)

    if image_condition:
        msg.data = np_image.reshape(-1).tolist()
    else:
        msg.data = np_image.reshape(-1)
    return msg

def ros_image_to_rgb_ndarray(image: Image) -> np.ndarray:
    H, W = image.height, image.width
    if image.encoding == 'rgb8':
        rgb = np.frombuffer(image.data, dtype=np.byte)
        rgb = rgb.reshape(H, W, 3).astype(np.uint8)
    elif image.encoding == 'bgra8':
        rgb = np.frombuffer(image.data, dtype=np.byte)
        rgb = rgb.reshape(H, W, 4)[:, :, (2,0,1)].astype(np.uint8)
    else: 
        raise RuntimeError('Not setup to handle incoming ros image as rgb ndarray.')
    return rgb

def ros_image_to_depth_ndarray(image: Image) -> np.ndarray:
    H, W = image.height, image.width
    if image.encoding == '16UC1':
        d = np.frombuffer(image.data, dtype=np.uint16).reshape(H, W)
    elif image.encoding == 'bgra8':
        rgbd = np.frombuffer(image.data, dtype=np.byte)
        rgbd = rgbd.reshape(H, W, 4)[:, :, (2,0,1)].astype(np.uint8)
        d = rgbd[:,:,3].astype(np.uint16)
    return d.astype(np.float32) / 1000

def get_xyz(d, K):
    W,H = d.shape[1], d.shape[0]
    vu = np.mgrid[:H, :W]
    ones = np.ones((1,H,W))
    uv1 = np.concatenate([vu[[1]], vu[[0]], ones], axis=0)
    uv1_prime = uv1 * d                                # [3, H, W]
    xyz = np.linalg.inv(K) @ uv1_prime.reshape(3, -1)  # [3,  HxW]
    return xyz

def images_to_pointcloud2(color:Image, depth:Image, K, transform) -> PointCloud2:

    
    assert(color.height == depth.height and color.width == depth.width)
    if (len(K) > 4): K = np.array(K).reshape(3,3)
    rgb = ros_image_to_rgb_ndarray(color)
    d = ros_image_to_depth_ndarray(depth)
    xyz = get_xyz(d, K).T / 10                         # [HxW, 3]

    depth_size = len(xyz)
    ones = np.ones(depth_size, float)
    hold = np.zeros((depth_size, 4))
    hold[:, :3] = xyz
    hold[:, 3] = ones
    distance = 0.19257
    # ex = np.deg2rad(180)
    # ey = np.deg2rad(-25)
    # ez = np.deg2rad(0)
    # T = affines.compose([0.22647, 0, 0.09967], euler.euler2mat(ex, ey, ez, 'rxyz'), [1, 1, 1])

    ex1 = np.deg2rad(90)
    ey1 = np.deg2rad(25)
    ez1 = np.deg2rad(0)
    # T1 = affines.compose([0.127+0.08, 0.105, 0], euler.euler2mat(ex1, ey1, ez1, 'rxyz'), [1, 1, 1])
    T1 = affines.compose([0.127+0.05, 0.105, 0], euler.euler2mat(ex1, ey1, ez1, 'rxyz'), [1, 1, 1])

    ex2 = np.deg2rad(-90)
    ey2 = np.deg2rad(25)
    ez2 = np.deg2rad(0)
    # T2 = affines.compose([0.127+0.08, -0.145, 0], euler.euler2mat(ex2, ey2, ez2, 'rxyz'), [1, 1, 1])
    T2 = affines.compose([0.127+0.05, -0.145, 0], euler.euler2mat(ex2, ey2, ez2, 'rxyz'), [1, 1, 1])
    
    if transform == 1:
        for i in range(depth_size):
            hold[i, :] = T1.dot(hold[i, :])
        hold = np.delete(hold, 3, 1)
        xyz = hold
        
    elif transform == 2:
        for i in range(depth_size):
            hold[i, :] = T2.dot(hold[i, :])
        hold = np.delete(hold, 3, 1)
        xyz = hold

    else:
        hold = np.delete(hold, 3, 1)
        xyz = hold    

    pcdL1 = xyz
    # Segment the center of the Gripper1
    # pcdL1[pcdL1[:, 2] < -.13] = 0
    # pcdL1[pcdL1[:, 2] > -.09] = 0
    # pcdL1[pcdL1[:, 1] < -.01] = 0
    # pcdL1[pcdL1[:, 1] > .01] = 0
    # pcdL1[pcdL1[:, 0] < -.02] = 0
    # pcdL1[pcdL1[:, 0] > .02] = 0
    # pcdL1 = pcdL1[pcdL1[:, 2] != 0]

    rgb = rgb.reshape(-1, 3)                            # [HxW, 3]
    header = color.header
    header.stamp = rospy.Time.now()
    return package_ros_pointcloud2(pcdL1, rgb, header)

def package_ros_pointcloud2(xyz, rgb, header):
    '''Given lists of xyz data and associated rgb returns pointcloud2 message'''
    fields = [PointField('x',   0,  PointField.FLOAT32, 1), 
              PointField('y',   4,  PointField.FLOAT32, 1), 
              PointField('z',   8,  PointField.FLOAT32, 1)
              ,PointField('rgba', 12, PointField.UINT32,  1)]

    a = np.ones((rgb.shape[0], 1)) * 255
    rgba = np.concatenate([rgb, a], axis=1)
    rgba = rgba.astype(np.uint8)
    xyz = xyz.astype(np.float32)

    points = []
    for i in range(len(xyz)):
        b, g, r, a = tuple(rgba[i])
        x, y, z = tuple(xyz[i])
        color = struct.unpack('I', struct.pack('BBBB', r, g, b, a))[0]
        pt = [x, y, z, color]
        points.append(pt)
    cloud = point_cloud2.create_cloud(header, fields, points)
    cloud.is_dense = True
    return cloud

def package_ros_pointcloud2_nrgb(xyz, header):
    '''Given lists of xyz data and associated rgb returns pointcloud2 message'''
    fields = [PointField('x',   0,  PointField.FLOAT32, 1), 
              PointField('y',   4,  PointField.FLOAT32, 1), 
              PointField('z',   8,  PointField.FLOAT32, 1)]

    xyz = xyz.astype(np.float32)

    points = []
    for i in range(len(xyz)):
        x, y, z = tuple(xyz[i])
        pt = [x, y, z]
        points.append(pt)
    cloud = point_cloud2.create_cloud(header, fields, points)
    cloud.is_dense = True
    return cloud

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

def config_bubble_cams() -> (rs.pipeline, rs.pipeline):
    # Configure depth and color streams...
    # ...from Camera 1
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device('126122270841')
    config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    
    #...from Camera 2
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device('126122270722')
    config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_2.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    print('Starting streaming...')

    # Start streaming
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)

    # Get frames
    frames_1 = pipeline_1.wait_for_frames()
    frames_2 = pipeline_2.wait_for_frames()
    color_frame_1 = frames_1.get_color_frame()
    color_frame_2 = frames_2.get_color_frame()

    # Get stream profile and camera intrinsics
    profile1 = pipeline_1.get_active_profile()
    profile2 = pipeline_2.get_active_profile()
    # profile3 = pipeline_3.get_active_profile()
    
    intrinsics1 = get_intrinsics(profile1.get_stream(rs.stream.depth))
    intrinsics2 = get_intrinsics(profile2.get_stream(rs.stream.depth))

    return (pipeline_1, pipeline_2)

def get_images (pipeline_1, pipeline_2) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray):
    
    # Cam 1
    frames_1 = pipeline_1.wait_for_frames()
    depth_frame_1 = frames_1.get_depth_frame()
    depth_image_1 = np.asanyarray(depth_frame_1.get_data())
    color_frame_1 = frames_1.get_color_frame()
    color_image_1 = np.asanyarray(color_frame_1.get_data())
    prev_1 = color_frame_1
    prev_1 = np.asanyarray(prev_1.get_data())
    prevgray_1 = cv2.cvtColor(prev_1, cv2.COLOR_BGR2GRAY)

    # Cam 2
    frames_2 = pipeline_2.wait_for_frames()
    depth_frame_2 = frames_2.get_depth_frame()
    depth_image_2 = np.asanyarray(depth_frame_2.get_data())
    color_frame_2 = frames_2.get_color_frame()
    color_image_2 = np.asanyarray(color_frame_2.get_data())
    prev_2 = color_frame_2
    prev_2 = np.asanyarray(prev_2.get_data())
    prevgray_2 = cv2.cvtColor(prev_2, cv2.COLOR_BGR2GRAY)

    return (depth_image_1, color_image_1, depth_image_2, color_image_2, prevgray_1, prevgray_2)