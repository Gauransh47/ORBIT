import open3d as o3d
import numpy as np


# Create a simple synthetic point cloud
points = np.random.uniform(
    low=[-5, -5, 0],
    high=[5, 5, 3],
    size=(5000, 3)
)

# Convert NumPy array → Open3D point cloud
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

# Display the point cloud
o3d.visualization.draw_geometries(
    [point_cloud],
    window_name="SIH LiDAR Prototype"
)