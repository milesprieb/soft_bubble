import bubble_helpers as bh
import contact_patch_estimator as cpe
import cv_utils as cvu
import flow_vis as fv
import numpy as np
import rospy

# ROS Imports
from sensor_msgs.msg import *
from sensor_msgs import point_cloud2
from visualization_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

def main():
    # Initialize ROS node
    rospy.init_node('bubble_gripper')

    # Configure cameras
    pipeline1, pipeline2 = bh.config_bubble_cams()

    while not rospy.is_shutdown():
        
        # Get depth and color images from cameras
        depth_image_1, color_image_1, depth_image_2, color_image_2, prevgray_1, prevgray_2 = bh.get_images(pipeline1, pipeline2)

        # Initalize publishers
        bubble_img_pub1 = rospy.Publisher("/bubble_rgb1", Image, queue_size=10)
        bubble_img_pub2 = rospy.Publisher("/bubble_rgb2", Image, queue_size=10)
        
        # Filling bubble images header
        bubble_image = Image()
        header_bubble = std_msgs.msg.Header()
        header_bubble.stamp = rospy.Time.now()
        header_bubble.frame_id = 'left_robotiq_85_base_link'
        bubble_image.header = header_bubble

        # Format images to ROS message
        ros_bubble_image_1 = bh.image_to_msg(color_image_1, bubble_image.header, image_condition=True)
        ros_bubble_image_2 = bh.image_to_msg(color_image_2, bubble_image.header, image_condition=True)

        # Publish bubble images
        bubble_img_pub1.publish(ros_bubble_image_1)
        bubble_img_pub2.publish(ros_bubble_image_2)

if __name__ == '__main__':
    main()