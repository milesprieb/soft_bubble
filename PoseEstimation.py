import numpy as np
import open3d as o3d

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
# Read in Left gripper point cloud
pcd = o3d.io.read_point_cloud('5hrv2.ply')
pcdL = np.asarray(pcd.points)

# Segment the center of the Gripper
pcdL[pcdL[:, 2] < -.13] = 0
pcdL[pcdL[:, 2] > -.09] = 0
pcdL[pcdL[:, 1] < -.01] = 0
pcdL[pcdL[:, 1] > .01] = 0
pcdL[pcdL[:, 0] < -.02] = 0
pcdL[pcdL[:, 0] > .02] = 0
pcdL = pcdL[pcdL[:, 2] != 0]


# Apply Transformation to point cloud
num1 = len(pcdL)
ones = np.ones(num1, float)
conv = np.zeros((num1, 4))
conv[:, :3] = pcdL
conv[:, 3] = ones
transform_1 = np.matrix([[.8192, 0, -0.5736, 0], [0, 1.0000, 0, 0], [0.5736, 0, 0.8192, 0], [0, 0, 0, 1.0000]])
for i in range(num1):
    conv[i, :] = transform_1.dot(conv[i, :])
conv = np.delete(conv, 3, 1)

# Transform right point cloud
distance = .11
transform_2 = np.matrix([[1, 0, 0, distance*.57357], [0, -1.0000, 0, 0], [0, 0, -1, -distance/.57357], [0, 0, 0, 1.0000]])
conv2 = np.zeros((num1, 4))
conv2[:, :3] = pcdL
conv2[:, 3] = ones
for i in range(num1):
    conv2[i, :] = transform_2.dot(conv2[i, :])
conv2 = np.delete(conv2, 3, 1)

# Concatenate Point clouds

length = len(conv)+len(conv2)
cat = np.zeros((length, 3))
cat[:len(conv), :] = conv
cat[len(conv2)::, :] = conv2

cat = cat * 1500

#ICP 1
trans_init = np.asarray([[0.862, 0.011, -0.507, 0],
                         [-0.139, 0.967, -0.215, 0],
                         [0.487, 0.255, 0.835, 0], [0.0, 0.0, 0.0, 1.0]])
source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(cat)
target = o3d.io.read_point_cloud('5hrenergy.ply')
targ = np.asarray(target.points)
h, w = targ.shape
remove = np.random.rand(h, w)
remove[remove < .8] = 0
remove[remove > .8] = 1
targ = np.multiply(targ, remove)
targ = targ[targ[:, 2] != 0]
targ = targ[targ[:, 0] != 0]
targ = targ[targ[:, 0] != 0]
target = o3d.geometry.PointCloud()
target.points = o3d.utility.Vector3dVector(targ)
threshold = 1000
reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=75))


#Transform
pcd = np.asarray(source.points)
num1 = len(pcd)
ones = np.ones(num1, float)
conv = np.zeros((num1, 4))
conv[:, :3] = pcd
conv[:, 3] = ones
transform_3 = reg_p2p.transformation
transform_4 = np.zeros((4, 4))
rot = [0, -90, 0]
rot = np.asarray(rot, dtype='float64')
transform_4[:3, :3] = o3d.geometry.get_rotation_matrix_from_xzy(rot)
transform_4[3, 3] = 1
for i in range(num1):
    conv[i, :] = transform_3.dot(conv[i, :])
for i in range(num1):
    conv[i, :] = transform_4.dot(conv[i, :])
conv = np.delete(conv, 3, 1)

# Use local points on Contact patch

source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(conv)

#ICP 2
threshold = 200
reg_p2p2 = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=400))
print(reg_p2p2)
print("Transformation is:")
print(reg_p2p2.transformation)
draw_registration_result(source, target, reg_p2p2.transformation)