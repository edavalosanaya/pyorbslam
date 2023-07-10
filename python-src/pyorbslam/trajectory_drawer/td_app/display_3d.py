from typing import Dict, Any
from PyQt5 import QtWidgets

import pyqtgraph.opengl as gl

class Display3D(gl.GLViewWidget):

    visuals: Dict[str, Any] # Dict[str, Item]

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        # Adding widgets
        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        # Container
        self.visuals = {}

    def add_visual(self):
        ...

    def update_visual(self):
        ...

    def delete_visual(self):
        ...

        # self.scatterItem = gl.GLScatterPlotItem(pos=np.empty((0, 3)), size=5, color=(1, 0, 0, 1))

        # self.addItem(self.scatterItem)

    # def setData(self, points, colors):
        # self.scatterItem.setData(pos=points, color=colors)

    # def onNewData(self):
        # # points = np.random.normal(size=(numPoints, 3))
        # points = np.random.uniform(low=0, high=1, size=(N,3))
        # colors = np.random.uniform(low=0, high=1, size=(N,4))
        # self.setData(points, colors)
