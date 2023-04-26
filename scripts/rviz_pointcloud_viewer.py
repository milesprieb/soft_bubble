# Imports
import numpy as np
import pyrealsense2 as rs
import rospy
import struct
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler
from transforms3d import *
import tf
import computeTF2frames
import aruco_detect
from scipy.spatial.transform import Rotation as R


# ROS msgs & services
from interactive_markers.interactive_marker_server import *
from sensor_msgs.msg import *
from sensor_msgs import point_cloud2
from visualization_msgs.msg import *
import std_msgs.msg
import geometry_msgs.msg
import open3d as o3d
import cv2 


def draw_registration_result(source, target, transformation):
    source_temp = source
    target_temp = target
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      height = 1080, width = 1920)
    

def makeBubble( msg ):
    marker = Marker()
    marker.header.frame_id = 'robotiq_85_base_link'
    marker.header.stamp = rospy.Time.now()
    marker.mesh_resource = 'file:///home/rpm/Code/realsense_test_ws/src/sample_pkg/meshes/dual_gripper.dae'
    marker.type = marker.MESH_RESOURCE
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 1
    marker.color.a = 1
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1

    return marker


def makeBubbleControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBubble(msg) )
    msg.controls.append( control )
    return control


def append_controls(int_marker):
    # x axis control
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    # x axis rotation
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    # y axis control
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    # y axis rotation
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    # z axis control
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    # z axis rotation
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)


def _image_to_msg(image, request_header, image_condition):
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


def _get_xyz(d, K):
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
    xyz = _get_xyz(d, K).T / 10                         # [HxW, 3]

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
    T1 = affines.compose([0.127+0.05, 0.13, 0], euler.euler2mat(ex1, ey1, ez1, 'rxyz'), [1, 1, 1])

    ex2 = np.deg2rad(-90)
    ey2 = np.deg2rad(25)
    ez2 = np.deg2rad(0)
    T2 = affines.compose([0.127+0.05, -0.13, 0], euler.euler2mat(ex2, ey2, ez2, 'rxyz'), [1, 1, 1])
    
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


def filter_stack(frame):
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    # Hole filling
    hole_filling = rs.hole_filling_filter()

    # Decimation - reduces depth frame density
    decimation = rs.decimation_filter()
    decimation.set_option(rs.option.filter_magnitude, 4)

    # Temporal
    temporal = rs.temporal_filter()

    # Spatial filter (edge-preserving)
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)

    frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = temporal.process(frame)
    frame = disparity_to_depth.process(frame)
    frame = hole_filling.process(frame)

    return frame


def icp_estimate(color1:Image, depth1:Image, color2:Image, depth2:Image, K1, K2, aruco_T_camera):

    assert(color1.height == depth1.height and color1.width == depth1.width)
    if (len(K1) > 4): K1 = np.array(K1).reshape(3,3)
    d1 = ros_image_to_depth_ndarray(depth1)
    xyz1 = _get_xyz(d1, K1).T / 10  

    assert(color2.height == depth2.height and color2.width == depth2.width)
    if (len(K2) > 4): K2 = np.array(K2).reshape(3,3)    
    d2 = ros_image_to_depth_ndarray(depth2)
    xyz2 = _get_xyz(d2, K2).T / 10
     

    pcdL1 = np.asarray(xyz1)
    pcdL2 = np.asarray(xyz2)

    ex1 = np.deg2rad(90)
    ey1 = np.deg2rad(25)
    ez1 = np.deg2rad(0)
    T1 = affines.compose([0.127+0.05, 0.13, 0], euler.euler2mat(ex1, ey1, ez1, 'rxyz'), [1, 1, 1])
    pcdL1 = (T1[:3, :3] @ pcdL1.T + T1[:3, 3].reshape(3, 1)).T

    ex2 = np.deg2rad(-90)
    ey2 = np.deg2rad(25)
    ez2 = np.deg2rad(0)
    T2 = affines.compose([0.127+0.05, -0.13, 0], euler.euler2mat(ex2, ey2, ez2, 'rxyz'), [1, 1, 1])
    pcdL2 = (T2[:3, :3] @ pcdL2.T + T2[:3, 3].reshape(3, 1)).T
    pcdL1 = pcdL1[pcdL1[:, 2] != 0]
    pcdL2 = pcdL2[pcdL2[:, 2] != 0]
    cat = np.concatenate((pcdL1, pcdL2), axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cat)

    #ICP 1
    trans_init = np.eye(4)                
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(cat)
    target = o3d.io.read_point_cloud('/home/rpm/Code/realsense_test_ws/src/sample_pkg/meshes/Initialized_champagne_new.ply')
    
    # visualize in open3d
    # o3d.visualization.draw_geometries([source, target],
    #                                 height = 1080, width = 1920)

    targ = np.asarray(target.points)
    num2 = len(targ)
    ones = np.ones(num2, float)
    trans_tar = np.zeros((num2, 4))
    trans_tar[:, :3] = targ
    trans_tar[:, 3] = ones
    # transform_4 = np.zeros((4, 4))
    # rot = [0, 0, 90]
    # rot = np.asarray(rot, dtype='float64')
    # transform_4[:3, :3] = o3d.geometry.get_rotation_matrix_from_xzy(rot)
    # transform_4[2, 3] = -0.1
    # transform_4[3, 3] = 1
    # for i in range(num2):
    #     trans_tar[i, :] = transform_4.dot(trans_tar[i, :])
    trans_tar = np.delete(trans_tar, 3, 1)
    rot_target = o3d.geometry.PointCloud()
    rot_target.points = o3d.utility.Vector3dVector(trans_tar)
    
 
    threshold = 0.5
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, rot_target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500))
    # draw_registration_result(source, rot_target, reg_p2p.transformation)
 
    # print("Transformation is:")
    # print(reg_p2p.transformation)
    # print(aruco_T_camera)
    icp_eval, obj_T_world = computeTF2frames.trans_eval(np.linalg.inv(reg_p2p.transformation), aruco_T_camera)
    print(icp_eval)
    #draw_registration_result(source, target, reg_p2p.transformation)
    
    source_T_target = np.linalg.inv(reg_p2p.transformation)
    rot_target.transform(obj_T_world)
    # 180 degree rotation about x axis
    # rot_target.rotate(np.asarray([[1, 0, 0], [0, -1, 0], [0, 0, -1]]))
    # rot_target.translate(np.asarray([0, 0, -0.1016]))
    # num2 = len(trans_tar)
    # ones = np.ones(num2, float)
    # trans_targ = np.zeros((num2, 4))
    # trans_targ[:, :3] = trans_tar
    # trans_targ[:, 3] = ones
    # for i in range(num2):
    #     trans_targ[i, :] = source_T_target.dot(trans_targ[i, :])
    # trans_targ = np.delete(trans_targ, 3, 1)

    return np.asarray(rot_target.points)



if __name__ == '__main__':

   
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

    #...from Camera 3
    pipeline_3 = rs.pipeline()
    config_3 = rs.config()
    config_3.enable_device('f1371463')
    config_3.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config_3.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)


    # Start streaming
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)
    pipeline_3.start(config_3)


    # Get stream profile and camera intrinsics
    profile1 = pipeline_1.get_active_profile()
    profile2 = pipeline_2.get_active_profile()
    profile3 = pipeline_3.get_active_profile()
    
    intrinsics1 = get_intrinsics(profile1.get_stream(rs.stream.depth))
    intrinsics2 = get_intrinsics(profile2.get_stream(rs.stream.depth))
    intrinsics3 = get_intrinsics(profile3.get_stream(rs.stream.color))


    rospy.init_node('bubble_gripper')

    bottle_marker = Marker()
    bottle_marker.header.frame_id = 'cam_1_depth_optical_frame'
    bottle_marker.header.stamp = rospy.Time.now()
    bottle_marker.mesh_resource = 'file:///home/rpm/Code/realsense_test_ws/src/sample_pkg/meshes/champagne_3d_z.dae'
    bottle_marker.type = bottle_marker.MESH_RESOURCE

    bubble_marker = Marker()
    bubble_marker.header.frame_id = 'robotiq_85_base_link'
    bubble_marker.header.stamp = rospy.Time.now()
    bubble_marker.mesh_resource = 'file:///home/rpm/Code/realsense_test_ws/src/sample_pkg/meshes/NEW_BG.dae'
    bubble_marker.type = bubble_marker.MESH_RESOURCE
   

    # create a marker
    # server = InteractiveMarkerServ400er("simple_marker")
    # int_marker = InteractiveMarker()
    # int_marker.header.frame_id = "world"
    # int_marker.name = "bubble"
    # # int_marker.description = "3-DOF Control"
    # int_marker.scale = 0.3
    # # insert a box
    # makeBubbleControl(int_marker)
    # int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    # append_controls(int_marker)
    # # tell the server to call processFeedback() when feedback arrives for it
    # server.insert(int_marker)
    # server.applyChanges()
    rate = rospy.Rate(10.0)
    norm_flag = True
    aruco_flag = True
    while not rospy.is_shutdown():
        # Wait for a coherent pair of frames: depth and color
        frames_1 = pipeline_1.wait_for_frames()
        frames_2 = pipeline_2.wait_for_frames()
        frames_3 = pipeline_3.wait_for_frames()

        # if norm_flag:
        #     normalized_right = np.zeros((480, 640))
        #     normalized_left = np.zeros((480, 640))
        #     for i in range(10):
        #         depth_frame = frames_1.get_depth_frame()
        #         depth_image_1 = np.asanyarray(depth_frame.get_data())
        #         depth_frame_2 = frames_2.get_depth_frame()
        #         depth_image_2 = np.asanyarray(depth_frame_2.get_data())

        #         depth_image_1[depth_image_1 > 1400] = 0
        #         depth_image_1[depth_image_1 < 1000] = 0
                
        #         depth_image_2[depth_image_2 > 1400] = 0
        #         depth_image_2[depth_image_2 < 1000] = 0

        #         normalized_right += depth_image_1
        #         normalized_left += depth_image_2

        #     normalized_right /= 10
        #     normalized_left /= 10
            
        #     norm_flag = False
        #     print("IMAGES NORMALIZED")

        depth_frame = frames_1.get_depth_frame()
        depth_image_1 = np.asanyarray(depth_frame.get_data())
        depth_frame_2 = frames_2.get_depth_frame()
        depth_image_2 = np.asanyarray(depth_frame_2.get_data())
        depth_frame_3 = frames_3.get_depth_frame()
        depth_image_3 = np.asanyarray(depth_frame_3.get_data())


        color_frame = frames_1.get_color_frame()
        color_frame_2 = frames_2.get_color_frame()
        color_frame_3 = frames_3.get_color_frame()

        color_image_1 = np.asanyarray(color_frame.get_data())
        color_image_2 = np.asanyarray(color_frame_2.get_data())
        color_image_3 = np.asanyarray(color_frame_3.get_data())


        # if aruco_flag:
        aruco_T_camera = aruco_detect.aruco_track(color_image_3, intrinsics3)
        # aruco_flag = False

        depth_image_1[depth_image_1 > 1500] = 0
        depth_image_1[depth_image_1 < 900] = 0
        depth_image_2[depth_image_2 > 1500] = 0
        depth_image_2[depth_image_2 < 900] = 0

        # depth_image_1 -= normalized_right.astype(np.uint16)
        # depth_image_2 -= normalized_left.astype(np.uint16)

 
        depth_image_1[:, :250] = 0
        depth_image_1[:, 350:] = 0
        depth_image_2[:, :250] = 0
        depth_image_2[:, 350:] = 0

        depth_image_1[:150, :] = 0
        depth_image_1[300:, :] = 0
        depth_image_2[:200, :] = 0
        depth_image_2[350:, :] = 0

        pointcloud_publisher1 = rospy.Publisher("/bubble_pcd1", PointCloud2, queue_size=10)
        pointcloud_publisher2 = rospy.Publisher("/bubble_pcd2", PointCloud2, queue_size=10)
        # pointcloud_publisher3 = rospy.Publisher("/top_down_pcd", PointCloud2, queue_size=10)
        images_publisher1 = rospy.Publisher("/bubble_rgb1", Image, queue_size=10)
        images_publisher2 = rospy.Publisher("/bubble_rgb2", Image, queue_size=10)
        object_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        object_publisher2 = rospy.Publisher("/visualization_marker2", Marker, queue_size = 2)
        target_publisher = rospy.Publisher("/target", PointCloud2, queue_size=10)
        images_publisher3 = rospy.Publisher("/top_down", Image, queue_size=10)

        #declaring pointcloud
        bubble_pcd = PointCloud2()
        #filling pointcloud header
        header_bubble = std_msgs.msg.Header()
        header_bubble.stamp = rospy.Time.now()
        header_bubble.frame_id = 'robotiq_85_base_link'
        bubble_pcd.header = header_bubble

        # #declaring pointcloud
        # top_down = Image()
        # # #filling pointcloud header
        # header_top = std_msgs.msg.Header()
        # header_top.stamp = rospy.Time.now()
        # header_top.frame_id = 'world'
        # top_down.header = header_top

        #filling pointcloud fields
        ros_bubble_image_1 = _image_to_msg(color_image_1, bubble_pcd.header, image_condition=True)
        ros_bubble_image_2 = _image_to_msg(color_image_2, bubble_pcd.header, image_condition=True)
        # ros_bubble_image_3 = _image_to_msg(color_image_3, bubble_pcd.header, image_condition=True)
        # top_down_img = _image_to_msg(color_image_3, top_down.header, image_condition=True)

        ros_bubble_rgb_1 = _image_to_msg(color_image_1, bubble_pcd.header, image_condition=False)
        ros_bubble_depth_1 = _image_to_msg(depth_image_1,bubble_pcd.header, image_condition=False)
        ros_bubble_rgb_2 = _image_to_msg(color_image_2, bubble_pcd.header, image_condition=False)
        ros_bubble_depth_2 = _image_to_msg(depth_image_2,bubble_pcd.header, image_condition=False)
        # ros_bubble_rgb_3 = _image_to_msg(color_image_3, top_down.header, image_condition=False)
        # ros_bubble_depth_3 = _image_to_msg(depth_image_3,top_down.header, image_condition=False)

        #declaring pointcloud
        targ_header = PointCloud2()

        #filling pointcloud header
        header_targ = std_msgs.msg.Header()
        header_targ.stamp = rospy.Time.now()
        header_targ.frame_id = 'world'
        targ_header.header = header_targ

        ros_target = icp_estimate(ros_bubble_rgb_1, ros_bubble_depth_1, ros_bubble_rgb_2, ros_bubble_depth_2, get_K(intrinsics1), get_K(intrinsics2), aruco_T_camera)
        ros_target_pub = package_ros_pointcloud2_nrgb(ros_target, targ_header.header)
        target_publisher.publish(ros_target_pub)
        # multiply by inverse of world_T_ee to get object_T_world
        #input(ee_T_object)
        #world_T_obj = world_T_ee @ ee_T_object

        #matrix to quaternion
        # q = tf.transformations.quaternion_from_matrix(source_T_target)

        #print(goal_pose)

        bubble_marker.color.r = 1
        bubble_marker.color.g = 0
        bubble_marker.color.b = 1
        bubble_marker.color.a = 1
        bubble_marker.scale.x = 1
        bubble_marker.scale.y = 1
        bubble_marker.scale.z = 1

        # 4x4 transformation matrix for 90 degree rotation around z-axis
        T1 = np.array([[0, 1, 0, 0],
                        [-1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        
        # 4x4 transformation matrix for 90 degree rotation around x-axis
        T2 = np.array([[0, 0, 1, 0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0],
                      [0, 0, 0, 1]])

        T = T1 @ T2

        q = tf.transformations.quaternion_from_matrix(T)
        
        bubble_marker.pose.position.x = 0.127 + 0.09
        bubble_marker.pose.position.y = 0
        bubble_marker.pose.position.z = 0
        bubble_marker.pose.orientation.x = q[0]
        bubble_marker.pose.orientation.y = q[1] 
        bubble_marker.pose.orientation.z = q[2]
        bubble_marker.pose.orientation.w = q[3]

        bottle_marker.color.r = 1
        bottle_marker.color.g = 0
        bottle_marker.color.b = 1
        bottle_marker.color.a = 1
        bottle_marker.scale.x = 1
        bottle_marker.scale.y = 1
        bottle_marker.scale.z = 1

        # get quat form euler
        # print(aruco_T_camera)
        q_bottle = tf.transformations.quaternion_from_euler(aruco_T_camera[3],  aruco_T_camera[4], aruco_T_camera[5], 'rxyz')
        bottle_marker.pose.position.x = aruco_T_camera[0]
        bottle_marker.pose.position.y = aruco_T_camera[1]
        bottle_marker.pose.position.z = aruco_T_camera[2]
        bottle_marker.pose.orientation.x = 0
        bottle_marker.pose.orientation.y = 0
        bottle_marker.pose.orientation.z = 0
        bottle_marker.pose.orientation.w = 1

        # aruco_gt = np.array([-0.13000626, -0.05819804 , 0.74811287, -1.8103643 , -2.46624047 ,-0.36967824])
        # aruco_gt = np.array([-0.19238228, -0.21368314,  1.22777117,  3.13533492, -0.17125897,  0.00505853])

        # q_bottle = tf.transformations.quaternion_from_euler(aruco_gt[3],  aruco_gt[4], aruco_gt[5], axes='rxyz')

        # bottle_marker.pose.position.x = aruco_gt[0]
        # bottle_marker.pose.position.y = aruco_gt[1]
        # bottle_marker.pose.position.z = aruco_gt[2]
        # bottle_marker.pose.orientation.x = q_bottle[0]
        # bottle_marker.pose.orientation.y = q_bottle[1]
        # bottle_marker.pose.orientation.z = q_bottle[2]
        # bottle_marker.pose.orientation.w = q_bottle[3]



        object_publisher.publish(bubble_marker)
        object_publisher2.publish(bottle_marker)
        bubble_pcd_pub_1 = images_to_pointcloud2(ros_bubble_rgb_1, ros_bubble_depth_1, get_K(intrinsics1), transform=2)
        bubble_pcd_pub_2 = images_to_pointcloud2(ros_bubble_rgb_2, ros_bubble_depth_2, get_K(intrinsics2), transform=1)
        # top_down = images_to_pointcloud2(ros_bubble_rgb_3, ros_bubble_depth_3, get_K(intrinsics3), transform=3)

        pointcloud_publisher1.publish(bubble_pcd_pub_1)
        pointcloud_publisher2.publish(bubble_pcd_pub_2)
        # pointcloud_publisher3.publish(top_down)
        images_publisher1.publish(ros_bubble_image_1)
        images_publisher2.publish(ros_bubble_image_2) 
        rospy.loginfo("publishing bubbles")
        # images_publisher3.publish(top_down_img)

        # # spin() simply keeps python from exiting until this node is stopped    
        # rospy.spin()
    
    # Stop streaming
    pipeline_1.stop()
    pipeline_2.stop()
    pipeline_3.stop()