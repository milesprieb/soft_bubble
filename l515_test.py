import pyrealsense2 as rs
import numpy as np
import cv2


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Align objects
align_to = rs.stream.color
align = rs.align(align_to)

# Start streaming
pipeline.start(config)


while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    color_image = np.asanyarray(color_frame.get_data())
    cv2.imshow('color', color_image)
    cv2.waitKey(1)
