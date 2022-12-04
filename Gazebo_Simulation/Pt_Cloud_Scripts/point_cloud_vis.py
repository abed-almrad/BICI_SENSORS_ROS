import numpy as np
import open3d as o3d
#Creating the numpy vector of points from a previously saved csv file
xyz = np.genfromtxt('/home/abed/.ros/inactive_taxels_coordinates1.csv',dtype= float, delimiter=',')
xyz = xyz[:,0:3]
inactive_pts = xyz[:,0:3]
inactive_color = np.full((inactive_pts.shape[0],3), 0.8)
#Creating a point cloud using the stored 3D points with the inactivity faded color
pcd_inactive = o3d.geometry.PointCloud()
pcd_inactive.points = o3d.utility.Vector3dVector(inactive_pts)
pcd_inactive.colors = o3d.utility.Vector3dVector(inactive_color)

#Creating the numpy vector of the active points from a previously saved csv file
xyz = np.genfromtxt('/home/abed/.ros/filtered_contacts_coordinates1.csv',dtype= float, delimiter=',')
active_pts = xyz[:,0:3]
active_color = np.full((active_pts.shape[0],3), 0.1)
#Creating a point cloud using the stored 3D points with the inactivity faded color
pcd_active = o3d.geometry.PointCloud()
pcd_active.points = o3d.utility.Vector3dVector(active_pts)
pcd_active.colors = o3d.utility.Vector3dVector(active_color)

#Point cloud visualization
o3d.visualization.draw_geometries([pcd_inactive,pcd_active])

#Creating a point cloud using the stored 3D points
#o3d.io.write_point_cloud("./data.ply", pcd)
#Reading and visualizing the data stored in a point cloud file
#pcd = o3d.io.read_point_cloud("./data.ply")
#o3d.visualization.draw_geometries([pcd])

# Visualization after voxels downsampling of the point cloud
#downpcd = pcd.voxel_down_sample(voxel_size=10)
#o3d.visualization.draw_geometries([downpcd])

#Visualization after the voxelization of the point cloud
#voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(downpcd, voxel_size=5)
#o3d.visualization.draw_geometries([voxel_grid])
#Surface reconstruction algorithms like: Alpha shapes, Ball pivoting and poisson surface reconstruction ... can be
#implemented to create shapes from unstructured point clouds (P.S. Check the o3d library API docs for more details 
#about these algorithms implementation)
