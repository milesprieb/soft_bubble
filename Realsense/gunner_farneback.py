#!/usr/bin/env python

'''
example to show optical flow

USAGE: opt_flow.py [<video_source>]

Keys:
 1 - toggle HSV flow visualization
 2 - toggle glitch

Keys:
    ESC    - exit
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2 as cv
import pyrealsense2 as rs


def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis


def draw_hsv(flow):
    h, w = flow.shape[:2]
    fx, fy = flow[:,:,0], flow[:,:,1]
    ang = np.arctan2(fy, fx) + np.pi
    v = np.sqrt(fx*fx+fy*fy)
    hsv = np.zeros((h, w, 3), np.uint8)
    hsv[...,0] = ang*(180/np.pi/2)
    hsv[...,1] = 255
    hsv[...,2] = np.minimum(v*4, 255)
    bgr = cv.cvtColor(hsv, cv.COLOR_HSV2BGR)
    return bgr


def warp_flow(img, flow):
    h, w = flow.shape[:2]
    flow = -flow
    flow[:,:,0] += np.arange(w)
    flow[:,:,1] += np.arange(h)[:,np.newaxis]
    res = cv.remap(img, flow, None, cv.INTER_LINEAR)
    return res

def main():
    import sys
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
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device('126122270722')
    config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    # Start streaming from both cameras
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)

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
    frames_2 = pipeline_2.wait_for_frames()
            
    
    depth_frame_2 = frames_2.get_depth_frame()
    color_frame_2 = frames_2.get_color_frame()
    prev_2 = color_frame_2

    prev_2 = np.asanyarray(prev_2.get_data())
    prevgray_2 = cv.cvtColor(prev_2, cv.COLOR_BGR2GRAY)
    show_hsv = False
    show_glitch = False
    cur_glitch = prev_2.copy()

    # Set up the Farneback optical flow parameters
    farneback_params = dict(pyr_scale=0.5,
                            levels=3,
                            winsize=15,
                            iterations=3,
                            poly_n=5,
                            poly_sigma=1.2,
                            flags=0)

    try:
        while True:

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
            frames_2 = pipeline_2.wait_for_frames()
            
            depth_frame_2 = frames_2.get_depth_frame()
            color_frame_2 = frames_2.get_color_frame()
            if not depth_frame_2 or not color_frame_2:
                continue
            prev_2 = color_frame_2

            # Convert images to numpy arrays
            depth_image_2 = np.asanyarray(depth_frame_2.get_data())
            color_image_2 = np.asanyarray(color_frame_2.get_data())

            #Flow Field Camera 2
            gray_2 = cv.cvtColor(color_image_2, cv.COLOR_BGR2GRAY)
            flow_2 = cv.calcOpticalFlowFarneback(prevgray_2, gray_2, None, **farneback_params)
            prevgray_2 = gray_2
            flipped_2 = cv.flip(draw_flow(gray_2, flow_2),1)


            prev_2 = np.asanyarray(prev_2.get_data())
            prevgray_2 = cv.cvtColor(prev_2, cv.COLOR_BGR2GRAY)
            show_hsv = False
            show_glitch = False
            cur_glitch = prev_2.copy()

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap_2 = cv.applyColorMap(cv.convertScaleAbs(depth_image_2, alpha=0.5), cv.COLORMAP_JET)

            # Stack all images horizontally
            images = np.hstack((flipped_1, flipped_2))

            # Show images from both cameras
            cv.namedWindow('RealSense', cv.WINDOW_NORMAL)
            cv.imshow('RealSense', images)
            cv.waitKey(1)

            # Save images and depth maps from both cameras by pressing 's'
            ch = cv.waitKey(25)
            if ch==115:
                cv.imwrite("my_image_1.jpg",color_image_1)
                cv.imwrite("my_depth_1.jpg",depth_colormap_1)
                cv.imwrite("my_image_2.jpg",color_image_2)
                cv.imwrite("my_depth_2.jpg",depth_colormap_2)
                print ("Save")


    finally:

        # Stop streaming
        pipeline_1.stop()
        pipeline_2.stop()


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()