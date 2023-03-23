import open3d as o3d
import numpy as np

def draw_registration_result(source, target, transformation):
    source_temp = source
    target_temp = target
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])
# read in files
# pcd = o3d.io.read_point_cloud('testing.ply')
pcd = o3d.io.read_point_cloud('5hrenergy1.ply')


#convert to np.array to manipulate data
pcd = np.asarray(pcd.points)
xyz = pcd
# filter points that are out of bounds
xyz[xyz[:, 2] < -.085] = 0
xyz_filtered = xyz[xyz[:, 2] != 0]
# xyz[xyz[:, 1] < -.01] = 0
# xyz[xyz[:, 1] > .01] = 0
# xyz[xyz[:, 0] > .045] = 0
# xyz[xyz[:, 0] < 0] = 0
# xyz_filtered = xyz[xyz[:, 1] != 0]
# convert back to point cloud and visualize
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz_filtered)
# o3d.visualization.draw_geometries([pcd], width=1920, height=1080)

pcd2 = o3d.io.read_point_cloud('5hrenergy2.ply')

# convert to np.array to manipulate data
pcd2 = np.asarray(pcd2.points)
xyz2 = pcd2

# filter points that are out of bounds
xyz2[xyz2[:, 2] < -.09] = 0

xyz2_filtered = xyz2[xyz2[:, 2] != 0]
# xyz2[xyz2[:, 0] < 0] = 0
#xyz2[xyz2[:, 0] > 0.05] = 0

xyz2[xyz2[:, 1] > .04] = 0
xyz2[xyz2[:, 1] < -.04] = 0
xyz2_filtered = xyz2[xyz2[:, 1] != 0]
print(xyz2_filtered.shape)


# convert back to point cloud and visualize
pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(xyz2_filtered)
print(pcd2.points)
# transform the first set
num1 = len(xyz2_filtered)
ones = np.ones(num1, float)
hold = np.zeros((num1, 4))
hold[:, :3] = xyz2_filtered
hold[:, 3] = ones
transform_1 = np.matrix([[.8192, 0, -0.5736, 0], [0, 1.0000, 0, 0], [0.5736, 0, 0.8192, 0], [0, 0, 0, 1.0000]])
for i in range(num1):
    hold[i, :] = transform_1.dot(hold[i, :])
hold = np.delete(hold, 3, 1)
cat_1 = hold
pcd3 = o3d.geometry.PointCloud()
pcd3.points = o3d.utility.Vector3dVector(cat_1)

# transform the second set
distance =.08
num2 = len(xyz2_filtered)
ones = np.ones(num2, float)
hold = np.zeros((num2, 4))
hold[:, :3] = xyz2_filtered
hold[:, 3] = ones
transform_2 = np.matrix([[1, 0, 0, distance*.57357], [0, -1.0000, 0, 0], [0, 0, -1, -distance/.57357], [0, 0, 0, 1.0000]])
T = transform_2
print(T)
for i in range(num2):
    hold[i, :] = T.dot(hold[i, :])
hold = np.delete(hold, 3, 1)
cat_2 = hold
pcd4 = o3d.geometry.PointCloud()
pcd4.points = o3d.utility.Vector3dVector(cat_2)

length = len(cat_1)+len(cat_2)
cat_3 = np.zeros((length, 3))
cat_3[:len(cat_1), :] = cat_1
cat_3[len(cat_1)::, :] = cat_2
cat_3 = cat_3

pcd5 = o3d.geometry.PointCloud()
pcd5.points = o3d.utility.Vector3dVector(cat_3)
o3d.visualization.draw_geometries([pcd5], width=1920, height=1080)


# o3d.visualization.draw_geometries([contact_patch], width=1920, height=1080)

source = pcd5
target = o3d.io.read_point_cloud('5hrenergy.ply')
targ = np.asarray(source.points)
targ = targ*1500
source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(targ)
trans_init = np.asarray([[0.862, 0.011, -0.507, 0],
                         [-0.139, 0.967, -0.215, 0],
                         [0.487, 0.255, 0.835, 0], [0.0, 0.0, 0.0, 1.0]])
#o3d.visualization.draw_geometries([target, source], width=1920, height=1080)

threshold = .02
reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
#draw_registration_result(source, target, reg_p2p.transformation)

bubble = o3d.io.read_point_cloud('Bubble Gripper Body.PLY')
bubble_t = np.asarray(bubble.points)
num1 = len(bubble_t)
ones = np.ones(num1, float)
hold = np.zeros((num1, 4))
hold[:, :3] = bubble_t
hold[:, 3] = ones
gamma = -90*3.14/180 #Z rotation
beta_rx = 0*3.14/180 # Y rotation
alpha_r = 35/2*3.14/180 # X rotation
T = np.matrix([[np.cos(beta_rx)*np.cos(gamma), np.sin(alpha_r)*np.sin(beta_rx)*np.cos(gamma)-np.cos(alpha_r)*np.sin(gamma), np.cos(alpha_r)*np.sin(beta_rx)*np.cos(gamma)+np.sin(alpha_r)*np.sin(gamma), .04],
     [np.cos(beta_rx)*np.sin(gamma), np.sin(alpha_r)*np.sin(beta_rx)*np.sin(gamma)+np.cos(alpha_r)*np.cos(gamma), np.cos(alpha_r)*np.sin(beta_rx)*np.sin(gamma)-np.sin(alpha_r)*np.cos(gamma), 0],
     [-np.sin(beta_rx), np.sin(alpha_r)*np.cos(beta_rx), np.cos(alpha_r)*np.cos(beta_rx), -0.04],
     [0, 0, 0, 1]])
gamma = 180*3.14/180 #Z rotation
beta_rx = 0*3.14/180 # Y rotation
alpha_r = 180*3.14/180 # X rotation
transform = np.matrix([[np.cos(beta_rx)*np.cos(gamma), np.sin(alpha_r)*np.sin(beta_rx)*np.cos(gamma)-np.cos(alpha_r)*np.sin(gamma), np.cos(alpha_r)*np.sin(beta_rx)*np.cos(gamma)+np.sin(alpha_r)*np.sin(gamma), 0],
     [np.cos(beta_rx)*np.sin(gamma), np.sin(alpha_r)*np.sin(beta_rx)*np.sin(gamma)+np.cos(alpha_r)*np.cos(gamma), np.cos(alpha_r)*np.sin(beta_rx)*np.sin(gamma)-np.sin(alpha_r)*np.cos(gamma), 0],
     [-np.sin(beta_rx), np.sin(alpha_r)*np.cos(beta_rx), np.cos(alpha_r)*np.cos(beta_rx), -0.05],
     [0, 0, 0, 1]])
transform = T*transform
for i in range(num1):
    hold[i, :] = T.dot(hold[i, :])
hold_2 = hold
hold = np.delete(hold, 3, 1)
cat_1 = hold


bubble = o3d.geometry.PointCloud()
bubble.points = o3d.utility.Vector3dVector(cat_1)
num1 = len(bubble_t)
ones = np.ones(num1, float)
hold = np.zeros((num1, 4))
hold[:, :3] = bubble_t
hold[:, 3] = ones
for i in range(num1):
    hold[i, :] = transform.dot(hold[i, :])
hold = np.delete(hold, 3, 1)
cat_2 = hold
bubble_2 = o3d.geometry.PointCloud()
bubble_2.points = o3d.utility.Vector3dVector(cat_2)
cat_3 = cat_3/1500
pcd6 = o3d.geometry.PointCloud()
pcd6.points = o3d.utility.Vector3dVector(cat_3)
o3d.visualization.draw_geometries([bubble, pcd5, bubble_2], width=1920, height=1080)


