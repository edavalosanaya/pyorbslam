import pathlib
import os
import logging
import yaml
import sys
from typing import Optional

from vispy import app, scene
import multiprocessing as mp
import numpy as np
import pyvista as pv

app.use_app(backend_name='glfw')

logger = logging.getLogger("pyorbslam")

N = 100

# Constants
DEFAULT_SETTINGS_PATH = pathlib.Path(os.path.abspath(__file__)).parent / 'settings.yaml'


class VispyApp:

    def __init__(self):

        # Imperative declearations
        self.queue: Optional[mp.Queue] = None

        # Configuring the plot
        self.canvas = scene.SceneCanvas(keys='interactive', show=True)
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = 'arcball'  # or try 'arcball'
        self.axis = scene.visuals.XYZAxis(parent=self.view.scene)

        # Adding elements
        self.scatter = scene.visuals.Markers()
        self.view.add(self.scatter)

        # Creating separate thread to check for updates and queue msgs
        self.timer = app.Timer()
        self.timer.connect(self.update)
        self.timer.start(0.05)
        
    def update(self, event):

        # If given queue
        # if self.queue:

        #     # Draw the msg
        #     try:
        #         msg = self.queue.get(block=False)
        #     except 
        
        pts = np.random.uniform(low=0, high=1, size=(N,3))
        color = np.random.uniform(low=0, high=1, size=(N,3))
        self.scatter.set_data(pts, edge_color=(0.5, 0.1, 0, .5), face_color=color, size=5)
        self.canvas.update()

    def run(self, queue: Optional[mp.Queue] = None):

        # Update the queue
        self.queue = queue

        # Run the app
        app.run()


class TrajectoryDrawer:

    def __init__(self):

        # Create the app
        self.vispy_app = VispyApp()
        self.queue = mp.Queue()

        # Start the VisPy application process
        self.vispy_proc = mp.Process(target=self.vispy_app.run, args=(self.queue,))
        self.vispy_proc.start()

    def __del__(self):
        self.vispy_proc.join()
