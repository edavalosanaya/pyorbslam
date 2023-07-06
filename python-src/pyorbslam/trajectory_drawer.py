import pathlib
import os
import logging
import yaml
from typing import Optional

import numpy as np
import pyvista as pv

from .slam import ASLAM
from .state import State

logger = logging.getLogger("pyorbslam")


# Constants
DEFAULT_SETTINGS_PATH = pathlib.Path(os.path.abspath(__file__)).parent / 'settings.yaml'


class TrajectoryDrawer:
    """This class is used to print a the trajectory result from the slam method in a sequence of images, it use the Plotily library """

    def __init__(
        self,
        params_file=DEFAULT_SETTINGS_PATH,
        drawpointcloud=True,
    ):
        """Build the Trajectory drawer

        Args:
            params_file (str): the Path to the .yaml file.
            width(int): the width of figure in pixel. Defaults to None
            height(int): the height of figure in pixel. Defaults to None
            drawpointcloud (bool): if is false the plot show only trajectory and not the point cloud. Defaults to True
            useFigureWidget (bool): use the plotily.graph_object.FigureWidget instance if false it used the plotily.graph_object.Figure
        """
        with open(params_file) as fs:
            fs.readline()
            self.params = yaml.safe_load(fs)

        # Extract the parameters
        self.eye_x = self.params['Drawer.eye.x']
        self.eye_y = self.params['Drawer.eye.y']
        self.eye_z = self.params['Drawer.eye.z']
        self.center_x = self.params["Drawer.center.x"]
        self.center_y = self.params["Drawer.center.y"]
        self.scale_grade_x = self.params["Drawer.scale_grade.x"]
        self.scale_grade_y = self.params["Drawer.scale_grade.y"]
        self.scale_grade_z = self.params["Drawer.scale_grade.z"]

        # Parameters
        self.drawpointcloud = drawpointcloud
        self.prec_camera_center = None

        # Storing mesh data
        self.camera_data = {'mesh': None, 'actor': None}

        # Create plotter
        self.plotter = pv.Plotter()
        self.plotter.show_axes()
        self.plotter.show(interactive_update=True)

    def create_camera_mesh(self):

        size_ratio = 10

        h = 0.5625 / size_ratio
        w = 1 / size_ratio
        d = 0.2 / size_ratio

        # Define the vertex coordinates of the mesh
        pointa = [w/2, h/2, d]
        pointb = [-w/2, h/2, d]
        pointc = [-w/2, -h/2, d]
        pointd = [w/2, -h/2, d]
        pointe = [0.0, 0.0, 0.0]
        return pv.Pyramid([pointa, pointb, pointc, pointd, pointe])

    def plot_trajectory(self, camera_pose: np.ndarray, pointcloud: Optional[np.ndarray] = None):
        """Compute the trajectory and add it to the figure
        """

        if isinstance(pointcloud, np.ndarray):

            # convert the camera coordinates to world coordinates
            cp, colors = zip(*pointcloud)
            wp = np.array([np.dot(camera_pose, point)[0:3] for point in cp]).reshape(-1, 3)

            # draw the point cloud
        
        # Drawing the path
        camera_center = camera_pose[0:3, 3].flatten()
        self.plotter.add_points(camera_center, color="green", render_points_as_spheres=True, point_size=6, opacity=0.5)

        # get the camera center in absolute coordinates
        if self.prec_camera_center is not None:
            logger.debug(f"Updating plotly's camera: {camera_pose}")
        
            # Draw the line from previous to current point
            line = pv.Line(self.prec_camera_center, camera_center)
            self.plotter.add_mesh(line, color="green", line_width=3, opacity=0.2)
           
            # Add the camera mesh
            camera_mesh = self.create_camera_mesh()
            camera_mesh.transform(camera_pose)

            if not self.camera_data['actor']:
                self.camera_data['actor'] = self.plotter.add_mesh(camera_mesh, color="red", style='wireframe', opacity=0.5)
            else:
                self.camera_data['actor'].mapper.SetInputData(camera_mesh)

        # Update camera center
        self.prec_camera_center = camera_center
        
        # Update the pyvista camera (position and focus)
        # self.plotter.set_position(
        #     [
        #         self.center_x + camera_center[0] * self.scale_grade_x,
        #         self.center_y + camera_center[1] * self.scale_grade_y,
        #         camera_center[2] * self.scale_grade_z
        #     ]
        # )

        self.plotter.set_focus(
            [
                self.eye_x + camera_center[0] * self.scale_grade_x,
                self.eye_y + camera_center[1] * self.scale_grade_y,
                self.eye_z + camera_center[2] * self.scale_grade_z
            ]
        )
        
        # Update the plot
        self.plotter.update()

        # return self.figure.layout.scene
