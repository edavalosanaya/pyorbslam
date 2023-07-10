import pathlib
import os
import logging
from functools import cached_property
from typing import Optional

from aiohttp import web
from PyQt5 import QtCore, QtWidgets
import multiprocessing as mp
import numpy as np
import pyqtgraph.opengl as gl

from .async_loop_thread import AsyncLoopThread

logger = logging.getLogger("pyorbslam")

N = 100

# Constants
DEFAULT_SETTINGS_PATH = pathlib.Path(os.path.abspath(__file__)).parent / 'settings.yaml'

class QtApp(QtWidgets.QApplication):

    def __init__(self, port: int = 9000):
        super().__init__([])

        # Save input parameters
        self.port = port

        # Setup the GUI
        self.window = MyWidget()
        self.window.resize(1280, 720)
        self.window.show()
        self.window.setWindowTitle("Lidar points")
        self.window.setCameraPosition(distance=40)
        self.window.raise_()
         
        # Setup the server
        self.setup_server()

    @cached_property
    def server(self):
        return web.Application()

    def setup_server(self):
        
        # Create the async loop thread
        self._thread = AsyncLoopThread()
        self._thread.start()

        # Adding routes
        self.server.add_routes([
            web.get("/", self.hello),
            web.get("/shutdown", self.shutdown)
        ])

        # Supporting the shutdown communication
        self.c = Communicate()
        self.c.closeApp.connect(self.window.close)

        # Run in an AsyncLoopThread
        self._thread.exec(self.start_server()).result(timeout=10)
        
    async def start_server(self):
        self._runner = web.AppRunner(self.server)
        await self._runner.setup()
        self._site = web.TCPSite(self._runner, "0.0.0.0", self.port)
        await self._site.start()

        self.port = self._site._server.sockets[0].getsockname()[1]
        logger.debug(f"Server running at localhost:{self.port}/")

    async def hello(self, request):
        return web.Response(text="Hello World!")

    async def shutdown(self, request):
        self.c.closeApp.emit()
        return web.Response(text="Shutting down")

    def exec_(self):
        super().exec_()
        self._thread.stop()

class Communicate(QtCore.QObject):
    closeApp = QtCore.pyqtSignal()

class MyWidget(gl.GLViewWidget):

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.scatterItem = gl.GLScatterPlotItem(pos=np.empty((0, 3)), size=5, color=(1, 0, 0, 1))

        self.addItem(self.scatterItem)

    def setData(self, points, colors):
        self.scatterItem.setData(pos=points, color=colors)

    def onNewData(self):
        # points = np.random.normal(size=(numPoints, 3))
        points = np.random.uniform(low=0, high=1, size=(N,3))
        colors = np.random.uniform(low=0, high=1, size=(N,4))
        self.setData(points, colors)


class TrajectoryDrawer:

    def __init__(self):

        # Create the app
        self.app =QtApp()

        # Start the VisPy application process
        self.app_proc = mp.Process(target=self.app.exec_)
        self.app_proc.start()

    def __del__(self):
        self.app_proc.join()
