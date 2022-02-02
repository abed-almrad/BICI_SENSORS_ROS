import numpy as np
import open3d as o3d
#Creating the numpy vector of points from a previously saved csv file
xyz = np.genfromtxt('points_coordinates.csv',dtype= float, delimiter=',')
xyz = xyz[:,0:3]

#Creating a point cloud using the stored 3D points
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)

#Point cloud visualization
o3d.visualization.draw_geometries([pcd])

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