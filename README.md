# Reusing Current Soft Bubble Gripper

## Building ROS Workspace for Soft Bubble Gripper
1. Create a workspace folder
2. Create directory 'src' 
3. Run 'catkin init'
4. Enter 'workspace/src'
5. Clone repo 
6. Go back to workspace
7. Run 'catkin build'

## Workspace Layout
```
soft bubble
│  CMakeLists.txt
|  README.md
|  package.xml
│
└───rviz 
    |
    ├── computeTF2frames.py
    ├── marker_test.py
    ├── point_cloud_publisher_example.py 
    ├── rviz_pointcloud_viewer.py
└───scripts
    │ 
    ├── bag_record.py
    ├── bubble_helpers.py
    ├── bubble_pub.py
    ├── contact_patch_estimator.py
    ├── cv_utils.py
    ├── flow_vis.py
    ├── gunner_farneback.py
└───tests
    ├── Realsense
    │   ├── dense_settings.json
    |
    ├── 3dviewer.py
    ├── aruco_detect.py
    ├── icp_estimate.py
```
## Where to start
1. Start by displaying RGB images and pointclouds from the Soft Bubble Grippers in RVIZ with:
```
python rviz/rviz_pointcloud_viewer.py
```
- This will publish RGB images to:
    - /bubble_rgb1
    - /bubble_rgb2
- This will publish pointclouds to:
    - /bubble_pcd1
    - /bubble_pcd2

2. Next try displaying the optical flow fields from the Soft Bubble Grippers use shear force thresholding to open the gripper when the shear force is above a certain threshold. This can be done with:
```
python scripts/gunnar_farneback.py
```


### rviz
- computeTF2frames.py: Computes the transform between two frames and publishes it to the TF2 tree. Used for computing the transform between the camera frame and the marker frame.
- marker_test.py: Publishes a test vmarker to the RViz visualization window
- point_cloud_publisher_example.py: Publishes a point cloud to the RViz visualization window
- rviz_pointcloud_viewer.py: Main script for running ICP experiments with the Soft Bubble Gripper. Displays rviz markers for the Soft Bubble Gripper and the object being grasped. Also displays the point cloud from the Intel RealSense L515 camera. Depends on [zeus](https://github.com/RPM-lab-UMN/zeus) to publish the robot model from:
```
roslaunch dual_arm zeus_bringup.launch
```

### scripts
- bag_record.py: Records a ROS bag of the RGB images from bubbles, optical flow, gripper state, and end-effector pose. Can be used to record demonstrations but must be used with [rosbag_data_utils](https://github.com/baldeeb/rosbag_data_utils) in order to train models with the data.
- bubble_helpers.py: Helper functions for visualizing pointclouds from the Soft Bubble Grippers
- bubble_pub.py: Publishes raw images from the soft bubbles to ROS topics
- contact_patch_estimator.py: Estimates the contact patch of the soft bubble gripper using the point cloud from the Intel RealSense D405s.
- cv_utils.py: Helper functions from Punyo
- flow_vis.py: Optical flow helper functions from Punyo
- gunner_farneback.py: TODO: Update this file

### tests
- Realsense
  - dense_settings.json: Config file to load settings into Intel Realsense Viewer for dense point cloud capture from the Intel D405
- aruco_detect.py: Detects ArUco markers from the Intel RealSense L515 and displays the pose of the marker in the camera frame
- aruco_track.png: Example image of ArUco marker used for tracking
- flow.mp4: Example video of optical flow from the Soft Bubble Gripper
- icp_estimate.py: Estimates the pose of the object being grasped by the Soft Bubble Gripper using ICP with *.ply files in Open3D. 


# Re-fabricating 
To fabricate the RPM Lab's current re-designed version of the Soft Bubble Grippers, follow along with these steps and images. The original version of the build instructions was used for reference and is located under: '/design files/Bubble Gripper Build Instructions v1.0.pdf'. Here are pictures of the steps you can follow along with:

![Soft Bubble V1.0 Fabrication Steps](/readme_files\images\fab.png?raw=true )
Top-left to bottom-right: (1) Painted latex pattern, mostly used for obtaining a dense depth maps in ICP experiments. Also used for obtaining visual features from deformation. (2) Method of applying spray adhesive to outside wring of acrylic window. (3) Adhering acrylic window to latex membrane. (4) Super gluing acrylic window and latex membrane to 3D-printed sealing band to ensure an air-tight bubble. (5) Re-designed 3D-printed parts including bubble mount, camera mount, and gripper mount. (6) Assembled Soft Bubble grippers.


## 1. Acrylic Window
- Download the DXF files for both sides of the acrylic window from '/design files/Acrylic Window Laser Cutter DXF Files/'
- Submit a work request here: [CSE Machine Shop Work Order Request](https://docs.google.com/forms/d/e/1FAIpQLSeT9CI8jEzGwpCazuahoLA_ba3dtILyQBaTtv5kv3Vj4mSTCQ/viewform?usp=send_form). Attach the DXF files and the build instructions file under the "New Fabrication" section in this ReadMe. You may need to meet with someone in the machine shop to walk them through the "Overview of Acrylic Window Laser Cut Process" in the build instructions. NOTE: The process must be done in two steps: 1) Have the machine shop cut the outline of the window. 2) Retrieve the square cutouts from the shop and install the heat inserts. This will ensure the pockets for them are not cut by accdient since the clearance is tight. Return them to the machine shop for the rest of the window to be cutout.
  
## 3D Printed Parts
- STL files for all 3D printed parts can be found under '/design files/3D Part Files/'
- This will include:
  - Bubble Mount
  - Gripper Mount
  - Camera Mount
  - Sealing Band
  - Pressure Sensing System
- Slice them and generate G code for your printer of choice. 

## Latex Membrane 
- Download the stencil for the latex membrane located here: '/design files/Latex Patterns/checkerboard.dxf'
- The '.dxf' file is compatible with the laser cutter in Anderson Labs, follow their official tutorial for how to use the laser cutter.
- There should be special settings for using paper in the machine (ask the lab manager if unsure), we used the shipping labels provided in the original bill of materials.
- Once the stencil is cut out, you can begin the painting process by taping a 14 cm x 18 cm piece of latex onto cardboard. MAKE SURE THE MATTE SIDE IS FACING UP.
- Tape the stencil on top of the latex and use the sponge paintbrush and the screen printing paint to paint over the stencil onto the latex. This step is labeled as (1) in the image above.
- Once you have covered the stencil, remove it and leave the latex to dry overnight.

## Bubble Module Assembly 
- To create the bubble module, follow the original build instructions under the 'Bubble Module Assembly'
- Make sure you tap the hole in the acrylic window with the M3 threaded tap before spray adhering to the latex. The spray adhesive step is shown in the image above as (2).
- Lay something heavy on the acrylic window while adhering to latex, only needs around 20-30 min to dry. The result of this step is labeled as (3) in the image above, this is after the sealing band has been adhered and left to dry.
- Make sure to leave a small excess of latex around the acrylic window for the sealing band to pinch (instructions should have amount to cut off and amount to leave). This part is tricky because it may be hard to fit into the sealing band due to the excess and uneveness. Apply extra super glue to areas that may have gaps after pressing into sealing band. This step is labeled as (4) in the image above.
- Lay something heavy on top of the acrylic window and sealing band overnight after you have superglued. 
- Pressurize the bubble with the bike pump through the push-to-connect nozzle to ensure there are no leaks. If there are leaks, you can try to apply more super glue to the area or you may need to start over.


## Soft Bubble Gripper Assembly
- By this point you should have the parts labeled in the image above as (5).
- To assemble the gripper, use the bolts corresponding to the heat sets in each piece. 
- Attaching the acrylic window to the bubble mount will take M2 washers and M2 x 4mm bolts.
- Attaching the bubble mount to the gripper mount will take 
- Attaching the camera mount to the gripper mount will take 
- The assembled grippers should look like the image above labeled as (6).

# New Fabrication

## [Soft Bubble Gripper Build Instructions v2.0](https://punyo.tech/downloads/Bubble_Gripper_Build_Instructions_v2.0.pdf)
This is the updated build instructions from Punyo at Toyota Research Insitute (TRI). This includes the end-to-end process from laser cutting the acrylic windows to stamping the latex membrane pattern. Laser cutting was outsourced to the CSE Machine Shop but all other fabrication can be done in-house.   

## [Soft Bubble Gripper Bill of Materials v2.0](https://punyo.tech/downloads/Bubble_Gripper_BOM_v2.0.pdf)
This is the updated bill of materials from Punyo at Toyota Research Insitute (TRI). This includes the quantity, pricing, and sourcing of all parts needed to fabricate new Soft Bubble Grippers.



