import numpy as np
import open3d as o3d
from threading import Thread
import time

N = 1000

# Create a point cloud
points = np.random.randn(N, 3)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Create the visualizer
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()

# Add the point cloud to the visualizer
vis.add_geometry(pcd)

try:
    while True:
        # Update the point cloud data
        new_points = np.random.randn(N, 3)
        pcd.points = o3d.utility.Vector3dVector(new_points)

        # Update the visualizer
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        # Sleep for a while
        time.sleep(0.5)
except:
    ...

# Close the visualizer window
vis.destroy_window()
