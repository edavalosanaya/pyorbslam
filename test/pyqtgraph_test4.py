
from PyQt5 import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import numpy as np
import sys

N = 52000


class Worker(QtCore.QObject):
    dataUpdated = QtCore.pyqtSignal(np.ndarray, np.ndarray)

    def __init__(self, parent=None):
        super().__init__(parent=parent)

    def generateData(self):
        points = np.random.uniform(low=0, high=1, size=(N, 3))
        colors = np.random.uniform(low=0, high=1, size=(N, 4))
        self.dataUpdated.emit(points, colors)


class MyWidget(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.scatterItem = gl.GLScatterPlotItem(pos=np.empty((0, 3)), size=5, color=(1, 0, 0, 1))
        self.addItem(self.scatterItem)

    def setData(self, points, colors):
        self.scatterItem.setData(pos=points, color=colors)


class App(QtCore.QObject):
    def __init__(self, args):
        super().__init__()

        self.app = QtWidgets.QApplication(args)
        self.worker = Worker()
        self.workerThread = QtCore.QThread()

        self.win = MyWidget()
        self.win.show()
        self.win.setWindowTitle("Lidar points")
        self.win.setCameraPosition(distance=40)

        self.worker.moveToThread(self.workerThread)
        self.worker.dataUpdated.connect(self.updateData)

        self.workerThread.started.connect(self.worker.generateData)
        self.workerThread.start()

    @QtCore.pyqtSlot(np.ndarray, np.ndarray)
    def updateData(self, points, colors):
        self.win.setData(points, colors)

    def exec(self):
        sys.exit(self.app.exec_())


def main():
    app = App([])

    app.exec()


if __name__ == "__main__":
    main()
