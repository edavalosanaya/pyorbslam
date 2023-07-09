#!/usr/bin/env python

from PyQt5 import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import numpy as np
N = 52000


class MyWidget(gl.GLViewWidget):

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)

        self.scatterItem = gl.GLScatterPlotItem(pos=np.empty((0, 3)), size=5, color=(1, 0, 0, 1))

        self.addItem(self.scatterItem)


    def setData(self, points, colors):
        self.scatterItem.setData(pos=points, color=colors)


    def onNewData(self):
        # points = np.random.normal(size=(numPoints, 3))
        points = np.random.uniform(low=0, high=1, size=(N,3))
        colors = np.random.uniform(low=0, high=1, size=(N,4))
        self.setData(points, colors)


def main():
    app = QtWidgets.QApplication([])

    win = MyWidget()
    win.show()
    win.setWindowTitle("Lidar points")
    win.setCameraPosition(distance=40)
    win.raise_()

    app.exec_()

if __name__ == "__main__":
    main()
