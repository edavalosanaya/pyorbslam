import logging
import multiprocessing as mp
from typing import Tuple, Union

import trimesh
import numpy as np

from ..tools import apply_rt_to_pts
from .td_app import TDApp
from .td_client import TDClient
from .data_container import MeshContainer, PointCloudContainer, LineContainer

logger = logging.getLogger("pyorbslam")


class TrajectoryDrawer:

    def __init__(self, port: int = 9000):

        # Create the app
        self.app = TDApp(port=port)

        # Start the VisPy application process
        self.app_proc = mp.Process(target=self.app.run)
        self.app_proc.start()

        # Create an HTTP Client
        self.client = TDClient(port=port)

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

        # Creating the correction matrix
        self._correction_rt = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])

    def _correct_pose(self, pose: np.ndarray):
        return np.matmul(self._correction_rt, pose)

    def _correct_pts(self, point_cloud: np.ndarray):
        return apply_rt_to_pts(point_cloud, self._correction_rt)

    #####################################################################################
    ## Trajectory Related Methods
    #####################################################################################

    def plot_line(self, line: np.ndarray, color: Union[Tuple[float, float, float, float], np.ndarray] = (1.0, 1.0, 1.0, 1.0), width: int = 1):

        # Correct the line to the visualization on the system
        correct_line = self._correct_pts(line)

        # Create container
        line_cont = LineContainer(
            pos=correct_line,
            color=color,
            width=width
        )
        
        if not 'path' in self.client.visuals:
            self.client.create_visual('path', 'line', line_cont)
        else:
            self.client.update_visual('path', 'line', line_cont)

    def plot_trajectory(self, pose: np.ndarray):

        # Apply a correct transformation
        pose = self._correct_pose(pose)

        # Extract the information here
        camera_center = pose[0:3, 3].reshape((1,3))
        self.trajectory_line = np.concatenate((self.trajectory_line, camera_center))

        # Create line container
        line_cont = LineContainer(
            pos=self.trajectory_line,
        )
       
        # Drawing the trajectory
        if not 'trajectory_line' in self.client.visuals:
            self.client.create_visual('trajectory_line', 'line', line_cont)
        else:
            self.client.update_visual('trajectory_line', 'line', line_cont)

        # Create Mesh Container
        fov_container = MeshContainer(
            mesh=self.fov_mesh.copy().apply_transform(pose),
            edgeColor=(1.0,0.0,0.0,1.0),
            drawFaces=False,
            drawEdges=True,
        )
        
        # Drawing the FOV
        if 'fov' not in self.client.visuals:
            self.client.create_visual('fov', 'mesh', fov_container)
        else:
            self.client.update_visual('fov', 'mesh', fov_container)

    def plot_image(self, image: np.ndarray):
        self.client.send_image(image)

    def plot_pointcloud(self, name: str, pts: np.ndarray, colors: Union[np.ndarray, Tuple[float, float, float, float]]=(1.0,1.0,1.0,1.0)):

        # Apply correction to the points
        pts = self._correct_pts(pts)

        # Create the container
        pc_container = PointCloudContainer(
            pts=pts,
            colors=colors
        )
        if name not in self.client.visuals:
            self.client.create_visual(name, 'point cloud', pc_container)
        else:
            self.client.update_visual(name, 'point cloud', pc_container)

    def reset(self):
        # Container information
        self.trajectory_line = np.empty((0,3))

        # Update the server via the client
        self.client.send_reset()
    
    #####################################################################################
    ## 3D Plotting
    #####################################################################################
   
    def add_mesh(self, name: str, mesh: trimesh.Trimesh, color:Tuple[float, float, float, float]=(1.0,1.0,1.0,1.0), edgeColor:Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0), drawFaces:bool=True, drawEdges:bool=False):

        if name in self.client.visuals:
            logger.warning(f"{self}: Cannot add mesh that is already added: {name}")
            return

        # Create container
        mesh_container = MeshContainer(
            mesh=mesh.apply_transform(self._correction_rt),
            color=color,
            edgeColor=edgeColor,
            drawFaces=drawFaces,
            drawEdges=drawEdges
        )
        self.client.create_visual(name, 'mesh', mesh_container)

    def update_mesh(self, name: str, mesh: trimesh.Trimesh, color:Tuple[float, float, float, float]=(1.0,1.0,1.0,1.0), edgeColor:Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0), drawFaces:bool=True, drawEdges:bool=False):
        
        if name not in self.client.visuals:
            logger.warning(f"{self}: Cannot update mesh that hasn't been added: {name}")
            return
        
        # Create container
        mesh_container = MeshContainer(
            mesh=mesh.apply_transform(self._correction_rt),
            color=color,
            edgeColor=edgeColor,
            drawFaces=drawFaces,
            drawEdges=drawEdges
        )
        self.client.update_visual(name, 'mesh', mesh_container)

    #####################################################################################
    ## Life Cycle
    #####################################################################################

    def stay(self):
        self.app_proc.join()

    def shutdown(self):
        self.client.shutdown()
        self.app_proc.join()

    def __del__(self):
        self.shutdown()
