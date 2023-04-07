import open3d as o3d
import numpy as np

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.zeros((1000,3)))
o3d.io.write_point_cloud("cloud_test.pcd",pcd,write_ascii=True)