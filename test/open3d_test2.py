import open3d as o3d
import numpy as np

def create_transform_matrix_from_z(z):
    """ Return transform 4x4 transformation matrix given a Z value """
    result = np.identity(4)
    result[2,3] = z # Change the z
    
    return result

# Create Open3d visualization window
vis = o3d.visualization.Visualizer()
vis.create_window()

# create sphere geometry
sphere1 = o3d.geometry.TriangleMesh.create_sphere()
vis.add_geometry(sphere1)

# create coordinate frame
coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
vis.add_geometry(coordinate_frame)

prev_tf = None
for curr_z in np.arange(0.5, 15.0, 0.005):
    # return sphere1 to original position (0,0,0)
    if prev_tf is not None:
        sphere1.transform(np.linalg.inv(prev_tf))

    # transform bazed on curr_z tf
    curr_tf = create_transform_matrix_from_z(curr_z)
    sphere1.transform(curr_tf)

    prev_tf = curr_tf

    vis.update_geometry(sphere1)
    vis.poll_events()
    vis.update_renderer()
