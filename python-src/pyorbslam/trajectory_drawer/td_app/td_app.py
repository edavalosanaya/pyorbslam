import pathlib
import os
import sys
import logging

from PyQt5 import QtWidgets

from ..utils import get_ip_address
from .comm_bus import CommBus
from .display_3d import Display3D
from .http_server import HttpServer
from .threaded_zmq_poller import ThreadedZmqPoller

logger = logging.getLogger("pyorbslam")

# Constants
DEFAULT_SETTINGS_PATH = pathlib.Path(os.path.abspath(__file__)).parent / 'settings.yaml'


class TDApp:

    def __init__(self, port: int = 9000):

        # Save input parameters
        self.port = port
        self.host = get_ip_address()

    def run(self):

        # Create the app
        self.app = QtWidgets.QApplication(sys.argv)

        # Creating communication bus
        self.cbus = CommBus()
          
        # Setup the GUI
        self.window = Display3D()
        self.window.resize(1280, 720)
        self.window.show()
        self.window.setWindowTitle("Lidar points")
        self.window.setCameraPosition(distance=40)
        self.window.raise_()
        
        # Supporting the shutdown communication
        self.cbus.dataUpdate.connect(self.window.update_visual)
        self.cbus.visualCreate.connect(self.window.create_visual)
        self.cbus.visualDelete.connect(self.window.delete_visual)
        self.cbus.closeApp.connect(self.window.close)

        # Setup ZeroMQ SUB
        self.zmq_poller = ThreadedZmqPoller(self.cbus)
        self.zmq_poller.start()
         
        # Setup the server
        self.server = HttpServer(self.port, self.cbus)
       
        # Run
        self.app.exec_()
        self.server.stop()
