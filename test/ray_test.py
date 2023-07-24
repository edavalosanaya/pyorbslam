import numpy as np
import trimesh

# Create a box with dimensions (x, y, z)
box = trimesh.creation.box(extents=[1, 1, 1])

# Define the origin of the ray
ray_origins = np.array([[0, 0, 2]])

# Define the direction of the ray
ray_directions = np.array([[0, 0, -1]])

# Find the intersection of the ray with the box
locations, index_ray, index_tri = box.ray.intersects_location(ray_origins, ray_directions)

# The intersection point is in the global coordinate system.
# If your box isn't centered at the origin, adjust the intersection point by the box's center.
intersection_point = locations[0] - box.centroid

# Now, normalize the intersection point coordinates to [0,1] range
# relative to the extents of the box in each dimension.
normalized_coordinates = (intersection_point + 0.5*box.extents) / box.extents

print(f"Normalized coordinates: {normalized_coordinates}")
