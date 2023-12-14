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
└───scripts
    │ 
└───tests
    |
```

# Re-fabricating 
To fabricate the RPM Lab's current re-designed version of the Soft Bubble Grippers, follow along with these steps and images. 

## 1. Acrylic Window
- Download the DXF files for both sides of the acrylic window from /design files/Acrylic Window Laser Cutter DXF Files
- Submit a work request here: [CSE Machine Shop Work Order Request](https://docs.google.com/forms/d/e/1FAIpQLSeT9CI8jEzGwpCazuahoLA_ba3dtILyQBaTtv5kv3Vj4mSTCQ/viewform?usp=send_form). Attach the DXF files and the build instructions file under the "New Fabrication" section in thie ReadMe. You may need to meet with someone in the machine shop to walk them through the "Overview of Acrylic Window Laser Cut Process" in the build instructions. NOTE: The process must be done in two steps: 1) Have the machine shop cut the outline of the window. 2) Retrieve the square cutouts from the shop and install the heat inserts. This will ensure the pockets for them are not cut by accdient since the clearance is tight. Return them to the machine shop for the rest of the window to be cutout.
  
## 3D Printed Parts
- STL files for all 3D printed parts can be found under /design files/3D Part Files
- This will include:
  - Bubble Mount
  - Gripper Mount
  - Camera Mount
  - Sealing Band
  - Pressure Sensing System
- Slice them and generate G code for your printer of choice. 

## Latex Membrane 
- Download tg

## Assembly

# New Fabrication

## [Soft Bubble Gripper Build Instructions v2.0](https://punyo.tech/downloads/Bubble_Gripper_Build_Instructions_v2.0.pdf)
This is the updated build instructions from Punyo at Toyota Research Insitute (TRI). This includes the end-to-end process from laser cutting the acrylic windows to stamping the latex membrane pattern. Laser cutting was outsourced to the CSE Machine Shop but all other fabrication can be done in-house.   

## [Soft Bubble Gripper Bill of Materials v2.0](https://punyo.tech/downloads/Bubble_Gripper_Build_Instructions_v2.0.pdf)
This is the updated bill of materials from Punyo at Toyota Research Insitute (TRI). This includes the quantity, pricing, and sourcing of all parts needed to fabricate new Soft Bubble Grippers.



