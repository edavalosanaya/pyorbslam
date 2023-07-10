import pathlib
import os
import logging
from typing import Optional

import multiprocessing as mp

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

    def draw(self):
        ...

    def shutdown(self):
        self.client.shutdown()
        self.app_proc.join()

    # def __del__(self):
