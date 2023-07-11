import logging
import multiprocessing as mp
from typing import Optional

import trimesh
import numpy as np

from .td_app import TDApp
from .td_client import TDClient
from .data_container import MeshContainer

logger = logging.getLogger("pyorbslam")


class TrajectoryDrawer:

    def __init__(self):

        # Create the app
        self.app = TDApp()

        # Start the VisPy application process
        self.app_proc = mp.Process(target=self.app.run)
        self.app_proc.start()

        # Create an HTTP Client
        self.client = TDClient()

        # Container information
        self.trajectory_line = np.empty((0,3))

        size_ratio = 10
        h = 0.5625 / size_ratio
        w = 1 / size_ratio
        d = 0.2 / size_ratio

        self.fov_mesh = trimesh.Trimesh(vertices=[
            [w/2,h/2,d],
            [-w/2,h/2,d],
            [-w/2,-h/2,d],
            [w/2, -h/2, d],
            [0, 0, 0]
        ], faces = [
            [0,1,2],
            [0,2,3],
            [1,3,2],
            [4,0,1],
            [4,2,1],
            [4,3,0],
            [4,3,2]
        ])

    def correct_pose(self, pose: np.ndarray):
        rt = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])

        return np.matmul(rt, pose)

    def plot_path(self, line: np.ndarray):
        
        if not 'path' in self.client.visuals:
            self.client.create_visual('path', 'line', line)
        else:
            self.client.update_visual('path', 'line', line)

    def plot_trajectory(self, pose: np.ndarray):

        # Apply a correct transformation
        pose = self.correct_pose(pose)

        # Extract the information here
        camera_center = pose[0:3, 3].reshape((1,3))
        self.trajectory_line = np.concatenate((self.trajectory_line, camera_center))
       
        # Drawing the trajectory
        if not 'trajectory_line' in self.client.visuals:
            self.client.create_visual('trajectory_line', 'line', self.trajectory_line)
        else:
            self.client.update_visual('trajectory_line', 'line', self.trajectory_line)

        # Drawing the FOV
        fov_container = MeshContainer(
            mesh=self.fov_mesh.copy().apply_transform(pose),
            drawFaces=False,
            drawEdges=True,
            color=(1,0,0,1)
        )
        if not 'fov' in self.client.visuals:
            self.client.create_visual('fov', 'mesh', fov_container)
        else:
            self.client.update_visual('fov', 'mesh', fov_container)

    def plot_image(self, image: np.ndarray):
        self.client.send_image(image)

    def plot_pointcloud(self):
        ...

    def stay(self):
        self.app_proc.join()

    def shutdown(self):
        self.client.shutdown()
        self.app_proc.join()

    def __del__(self):
        self.shutdown()
