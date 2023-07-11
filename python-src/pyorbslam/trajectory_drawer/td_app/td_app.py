import pathlib
import os
import sys
import logging

from PyQt5 import QtWidgets
import pyqtgraph.opengl as gl

from ..utils import get_ip_address
from .comm_bus import CommBus
from .display_3d import Display3D
from .display_image import DisplayImage
from .http_server import HttpServer
from .threaded_zmq_poller import ThreadedZmqPoller

logger = logging.getLogger("pyorbslam")

# Constants
DEFAULT_SETTINGS_PATH = pathlib.Path(os.path.abspath(__file__)).parent / 'settings.yaml'

class AppWindow(gl.GLViewWidget):

    def __init__(self, port: int):
        super().__init__()

        # Storing input parameters
        self.port = port
        
        # Creating communication bus
        self.cbus = CommBus()
        
        # Setup the widgets
        self.display3d = Display3D()
        self.display3d.setCameraPosition(distance=40)
        self.display_image = DisplayImage()

        self.mainLayout = QtWidgets.QHBoxLayout()
        self.setLayout(self.mainLayout)

        self.mainLayout.addWidget(self.display3d)
        self.mainLayout.addWidget(self.display_image)

        self.mainLayout.setStretchFactor(self.display3d, 1)
        self.mainLayout.setStretchFactor(self.display_image, 1)

        # Setup the server
        self.server = HttpServer(self.port, self.cbus)
        
        # Setup ZeroMQ SUB
        self.zmq_poller = ThreadedZmqPoller(self.cbus)
        self.zmq_poller.start()
        
        # Supporting the shutdown communication
        self.cbus.dataUpdate.connect(self.display3d.update_visual)
        self.cbus.visualCreate.connect(self.display3d.create_visual)
        self.cbus.visualDelete.connect(self.display3d.delete_visual)
        self.cbus.closeApp.connect(self.closeEvent)
        self.cbus.closeApp.connect(self.server.stop) 

class TDApp:

    def __init__(self, port: int = 9000):

        # Save input parameters
        self.port = port
        self.host = get_ip_address()

    def run(self):

        # Create the app
        self.app = QtWidgets.QApplication(sys.argv)
        self.window = AppWindow(self.port)
        self.window.resize(1280, 720)
        self.window.setWindowTitle("Lidar points")
        self.window.raise_()
        self.window.show()
 
        # Run
        self.app.exec_()
