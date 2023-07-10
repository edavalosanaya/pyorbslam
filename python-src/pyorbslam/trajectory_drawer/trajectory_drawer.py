import logging
import multiprocessing as mp
from typing import Optional

import numpy as np

from .td_app import TDApp
from .td_client import TDClient

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

    def plot_trajectory(self, pose: np.ndarray):

        # Extract the information here
        camera_center = pose[0:3, 3].flatten()
        self.trajectory_line = np.append(self.trajectory_line, camera_center)[-30:, :]
        
        if not 'trajectory_line' in self.client.visuals:
            self.client.create_visual('trajectory_line', 'line', self.trajectory_line)
        else:
            self.client.update_visual('trajectory_line', 'line', self.trajectory_line)

    def plot_pointcloud(self):
        ...

    def shutdown(self):
        self.client.shutdown()
        self.app_proc.join()

    def __del__(self):
        self.shutdown()
