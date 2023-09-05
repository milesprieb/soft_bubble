# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2 as cv
import pyrealsense2 as rs
import sys
import rospy
import std_msgs.msg
import matplotlib.pyplot as plt
import tf
import time
from visualization_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *


def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int) #30x40 dot pattern
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def draw_flow_ltx(img, flow, dots, step=16):
    if len(dots) > 0:
        y, x = zip(*dots)
    #print(len(y),len(x))
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis


def dot_locate(img):
    image = img
    original = image.copy()
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray, (3,3), 0)
    th1 = cv.threshold(blur, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)[1]
    th2 = cv.adaptiveThreshold(gray,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,11,2)

    # Find contours: https://learnopencv.com/contour-detection-using-opencv-python-c/
    cnts = cv.findContours(th2, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    #print(np.shape(cnts), np.shape(cnts[0]), np.shape(cnts[1]))
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    M = cv.moments(cnts[0])
    if M["m00"] == 0:
        M["m00"] = -np.inf
    cX = int(M["m10"] / M["m00"]) # Finding center of massess for all dots: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html#ab8972f76cccd51af351cbda199fb4a0d
    cY = int(M["m01"] / M["m00"])
    x, y, z = np.where(img==(320, 159, 22))
    points = zip(x,y)   

    return points
    
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

def main():
    
    # Initialize ROS node
    rospy.init_node('flow_field')

    # Initialize publisher
    shear_publisher = rospy.Publisher('shear_force_pub', std_msgs.msg.Float32, queue_size=10)

    try:
        fn = sys.argv[1]
    except IndexError:
        fn = 0

    # Configure depth and color streams...
    # ...from Camera 1
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device('126122270841')
    config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # ...from Camera 2
    # pipeline_2 = rs.pipeline()
    # config_2 = rs.config()
    # config_2.enable_device('126122270722')
    # config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    #config_2.enable_record_to_file('flow_field.bag')


    # Start streaming from both cameras
    pipeline_1.start(config_1)
    # pipeline_2.start(config_2)

    # Camera 1
    frames_1 = pipeline_1.wait_for_frames()
            
    
    depth_frame_1 = frames_1.get_depth_frame()
    color_frame_1 = frames_1.get_color_frame()
    prev_1 = color_frame_1

    prev_1 = np.asanyarray(prev_1.get_data())
    prevgray_1 = cv.cvtColor(prev_1, cv.COLOR_BGR2GRAY)
    show_hsv = False
    show_glitch = False
    cur_glitch = prev_1.copy()

    # Camera 2
    # frames_2 = pipeline_2.wait_for_frames()
            
    
    # depth_frame_2 = frames_2.get_depth_frame()
    # color_frame_2 = frames_2.get_color_frame()
    # prev_2 = color_frame_2

    # prev_2 = np.asanyarray(prev_2.get_data())
    # prevgray_2 = cv.cvtColor(prev_2, cv.COLOR_BGR2GRAY)
    # show_hsv = False
    # show_glitch = False
    # cur_glitch = prev_2.copy()

    # Set up the Farneback optical flow parameters
    farneback_params = dict(pyr_scale=0.5,
                            levels=3,
                            winsize=15,
                            iterations=3,
                            poly_n=5,
                            poly_sigma=1.2,
                            flags=0)

    try:
        fourcc = cv.VideoWriter_fourcc(*'mp4v')
        out = cv.VideoWriter('flow.mp4',fourcc, 10.0, (640,480))
        count = 0
        flow_angles = []
        while not rospy.is_shutdown():

            # Camera 1
            # Wait for a coherent pair of frames: depth and color
            frames_1 = pipeline_1.wait_for_frames()
            
    
            depth_frame_1 = frames_1.get_depth_frame()
            color_frame_1 = frames_1.get_color_frame()
            prev_1 = color_frame_1
            if not depth_frame_1 or not color_frame_1:
                continue

            # Convert images to numpy arrays
            depth_image_1 = np.asanyarray(depth_frame_1.get_data())
            color_image_1 = np.asanyarray(color_frame_1.get_data())

            #Flow Field Camera 1
            gray_1 = cv.cvtColor(color_image_1, cv.COLOR_BGR2GRAY)
            flow_1 = cv.calcOpticalFlowFarneback(prevgray_1, gray_1, None, **farneback_params)
            # get the direction angle of the flow field 
            # Apply Gaussian filter to flow vectors
            filtered_flow = cv.GaussianBlur(flow_1, (0, 0), 3)

            flow_mag, flow_ang = cv.cartToPolar(filtered_flow[..., 0], filtered_flow[..., 1])
            avg_flow_ang = np.mean(flow_ang)
            avg_flow_mag = np.mean(flow_mag)
            print(avg_flow_mag)
            # print(np.max(flow_ang))
            x_mean = np.mean(filtered_flow[..., 0]) 
            y_mean = np.mean(filtered_flow[..., 1])
            # print(x_mean, y_mean)
            if avg_flow_mag > 1.0:
                # print(x_mean, y_mean)
                if x_mean > 0 and y_mean > 0:
                    # Scale to various ranges corresponding to quadrants: https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
                    # ( (old_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min
                    avg_flow_ang = ( (avg_flow_ang - 0) / (2*np.pi - 0) ) * (np.pi/2 - 0) + 0
                    # print(f'x mean: {x_mean}, y mean: {y_mean}, flow angle: {avg_flow_ang}')
                elif x_mean < 0 and y_mean > 0:
                    avg_flow_ang = ( (avg_flow_ang - 0) / (2*np.pi - 0) ) * (np.pi - np.pi/2) + np.pi/2
                    # print(f'x mean: {x_mean}, y mean: {y_mean}, flow angle: {avg_flow_ang}')
                elif x_mean < 0 and y_mean < 0:
                    avg_flow_ang = ( (avg_flow_ang - 0) / (2*np.pi - 0) ) * (3*np.pi/2 - np.pi) + np.pi
                    # print(f'x mean: {x_mean}, y mean: {y_mean}, flow angle: {avg_flow_ang}')
                elif x_mean > 0 and y_mean < 0:
                    avg_flow_ang = ( (avg_flow_ang - 0) / (2*np.pi - 0) ) * (2*np.pi - 3*np.pi/2) + 3*np.pi/2
                    # print(f'x mean: {x_mean}, y mean: {y_mean}, flow angle: {avg_flow_ang}')
                # Calculate overall flow direction
                # print(avg_flow_ang)

            # avg_flow_ang_rad = np.radians(avg_flow_ang)
            # if avg_flow_ang_rad < -np.pi:
            #     avg_flow_ang_rad += 2 * np.pi

            # time.sleep(0.5)
            # print(np.degrees(avg_flow_ang))
            # print(f'Magnitude: {np.mean(mag)}, Angle: {np.mean(angle_rad * 180/np.pi)}')
            # plt.scatter(count, np.mean(np.mean(ang * 180/np.pi)))
            # plt.pause(0.05)
            count += 1            

            prevgray_1 = gray_1
            
            flipped_1 = cv.flip(draw_flow(gray_1, flow_1),1)

            prev_1 = np.asanyarray(prev_1.get_data())
            prevgray_1 = cv.cvtColor(prev_1, cv.COLOR_BGR2GRAY)
            show_hsv = False
            show_glitch = False
            cur_glitch = prev_1.copy()

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap_1 = cv.applyColorMap(cv.convertScaleAbs(depth_image_1, alpha=0.5), cv.COLORMAP_JET)

            # Camera 2
            # Wait for a coherent pair of frames: depth and color
            # frames_2 = pipeline_2.wait_for_frames()
            
            # depth_frame_2 = frames_2.get_depth_frame()
            # color_frame_2 = frames_2.get_color_frame()
            # if not depth_frame_2 or not color_frame_2:
            #     continue
            # prev_2 = color_frame_2

            # # Convert images to numpy arrays
            # depth_image_2 = np.asanyarray(depth_frame_2.get_data())
            # color_image_2 = np.asanyarray(color_frame_2.get_data())

            #Flow Field Camera 2
            # gray_2 = cv.cvtColor(color_image_2, cv.COLOR_BGR2GRAY)
            # flow_2 = cv.calcOpticalFlowFarneback(prevgray_2, gray_2, None, **farneback_params) # 480x640
            
        
            # prevgray_2 = gray_2
            # flipped_2 = cv.flip(draw_flow(gray_2, flow_2),1)


            # prev_2 = np.asanyarray(prev_2.get_data())
            # prevgray_2 = cv.cvtColor(prev_2, cv.COLOR_BGR2GRAY)
            # show_hsv = False
            # show_glitch = False
            # cur_glitch = prev_2.copy()

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            # depth_colormap_2 = cv.applyColorMap(cv.convertScaleAbs(depth_image_2, alpha=0.5), cv.COLORMAP_JET)

            # Stack all images horizontally
            # images = np.hstack((flipped_1, flipped_2))

            bubble_pcd = Image()
            #filling pointcloud header
            header_bubble = std_msgs.msg.Header()
            header_bubble.stamp = rospy.Time.now()
            header_bubble.frame_id = 'left_robotiq_85_base_link'
            bubble_pcd.header = header_bubble

            ros_bubble_image_1 = _image_to_msg(flipped_1, bubble_pcd.header, image_condition=True)
            # ros_bubble_image_2 = _image_to_msg(flipped_2, bubble_pcd.header, image_condition=True)

            images_publisher1 = rospy.Publisher("/bubble_rgb1", Image, queue_size=10)
            images_publisher2 = rospy.Publisher("/bubble_rgb2", Image, queue_size=10)
            arrow_publisher = rospy.Publisher("/arrow", Marker, queue_size=10)

            images_publisher1.publish(ros_bubble_image_1)
            # images_publisher2.publish(ros_bubble_image_2)

            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            marker.type = 0
            marker.id = 0
           
            # print(f'Marker scale: {marker_scale}')
            marker.scale.x = avg_flow_mag
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Set the color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Set the pose of the marker
            # if count == 5: 
            # flow_ang_mean = np.mean(flow_angles)    
            q = tf.transformations.quaternion_from_euler(0, 0, avg_flow_ang)
            q = q/np.linalg.norm(q)
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]

            arrow_publisher.publish(marker)

            #     flow_angles = []
            #     count = 0
            # else:
            #     flow_angles.append(np.mean(avg_flow_ang))
            # q = np.normalized(q)

            

            # dots = dot_locate(color_image_1)
            # res = cv.flip(draw_flow_ltx(gray_1, flow_1, dots),1)

            # Write to video file
            # out.write(res)

            # # Check if slip is detected
            # # slip_cond = np.mean(flow_1) + np.mean(flow_2)
            # if slip_cond > 1.0:
            #     rospy.loginfo("Slip detected!")
            #     shear_publisher.publish(slip_cond)

            

            # Show images from both cameras
            cv.namedWindow('RealSense', cv.WINDOW_NORMAL)
            cv.imshow('RealSense', flipped_1)
            cv.waitKey(1)
            
            # Declare pointcloud object, for calculating pointclouds and texture mappings
            pc = rs.pointcloud()
            
            # We want the points object to be persistent so we can display the last cloud when a frame drops
            points = rs.points()

            # Save images and depth maps from both cameras by pressing 's'
            ch = cv.waitKey(25)
            if ch==115:
                cv.imwrite("my_image_1.jpg",color_image_1)
                cv.imwrite("my_depth_1.jpg",depth_colormap_1)
                # cv.imwrite("my_image_2.jpg",color_image_2)
                # cv.imwrite("my_depth_2.jpg",depth_colormap_2)
                print ("Save")

            if cv.waitKey(1) & 0xFF == ord('s'):
                # plt.show()
                break
             
    finally:
  
        # Stop streamings
        pipeline_1.stop()
        out.release()
        # pipeline_2.stop()
        

#test

if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()