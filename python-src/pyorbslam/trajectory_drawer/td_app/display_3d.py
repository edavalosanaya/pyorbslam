import logging
from typing import Dict, Any, Literal

import numpy as np
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QColor
import pyqtgraph.opengl as gl

logger = logging.getLogger('pyorbslam')

from ..data_chunk import DataChunk

class Display3D(gl.GLViewWidget):

    visuals: Dict[str, Any] # Dict[str, Item]

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        # Adding widgets
        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        # Container
        self.visuals = {}

        # Map for creating empty elements
        self.item_create_map = {
            'line': self.create_line
        }

        self.item_update_map = {
            'line': self.update_line
        }
        
        # Changing the defaults
        # self.setBackgroundColor(QColor(255,255,255))
        
        # Create an AxisItem for the bottom axis
        bottom_axis = gl.GLAxisItem()
        self.addItem(bottom_axis)
        self.visuals['axis'] = bottom_axis

        self.scatterItem = gl.GLScatterPlotItem(pos=np.empty((0,3)), size=5, color=(1, 0, 0, 1))

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.setData)
        self.timer.start(1000)  # Run every 1 second
    
    def setData(self):
        # Your custom code here
        logger.debug(f"{self}: setData!")
        N = 100
        points = np.random.uniform(low=0, high=1, size=(N, 3))
        colors = np.random.uniform(low=0, high=1, size=(N, 4))
        self.scatterItem.setData(pos=points, color=colors)

    def __str__(self):
        return "<Display3D>"

    def __repr__(self):
        return str(self)

    def create_visual(self, name: str, vtype: Literal['line']):
        
        # Create the item
        item = self.item_create_map[vtype]()

        # Then storing the data
        self.visuals[name] = item

        logger.debug(f"{self}: Created visual: {name}")

    # def setData(self, data_chunk: DataChunk):
    #     logger.debug(f"{self}::setData: {data_chunk.name} - {data_chunk.vtype}")
    #     update_fun = self.item_update_map[data_chunk.vtype]
    #     update_fun(self.visuals[data_chunk.name], data_chunk)
    #     logger.debug(f"{self}::setData: {data_chunk.name} - {data_chunk.vtype} - FINISHED")

    def delete_visual(self, name: str):

        # Obtain and delete
        item = self.visuals[name]
        self.removeItem(item)
        
        logger.debug(f"{self}: Deleted visual: {name}")

    ####################################################################################
    ## Creating visual elements
    ####################################################################################

    def create_line(self):
        return gl.GLLinePlotItem(pos=np.empty((0,3)))

    def update_line(self, line, data_chunk: DataChunk):
        logger.debug(f"{self}: {data_chunk.data.shape}")
        line.setData(pos=data_chunk.data, color=QColor(255,0,0,1), width=15)
