import numpy as np
import k3d
import time

N = 1000

# Generate initial random points
points = np.random.randn(N, 3)
plot = k3d.plot()

# Create the point cloud plot
point_cloud = k3d.points(points, color=0xff0000, point_size=0.2)
plot += point_cloud

# Set up the visualization
plot.display()

# Update the points in a loop
while True:
    # Generate new random points
    new_points = np.random.randn(N, 3)

    # Update the point cloud data
    point_cloud.positions = new_points

    # Render the updated plot
    plot.fetch_screenshot()
    time.sleep(0.5)

