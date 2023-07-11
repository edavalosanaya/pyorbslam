from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np

app = QtWidgets.QApplication([])
w = gl.GLViewWidget()
w.showMaximized()
w.setWindowTitle('pyqtgraph example: GLMeshItem')
w.setCameraPosition(distance=40)

g = gl.GLGridItem()
g.scale(2,2,1)
w.addItem(g)

verts = np.array([
    [0, 0, 0],
    [2, 0, 0],
    [1, 2, 0],
    [1, 1, 1],
])
faces = np.array([
    [0, 1, 2],
    [0, 1, 3],
    [0, 2, 3],
    [1, 2, 3]
])
colors = np.array([
    [1, 0, 0, 0.3],
    [0, 1, 0, 0.3],
    [0, 0, 1, 0.3],
    [1, 1, 0, 0.3]
])


md = gl.MeshData.sphere(rows=4, cols=4)

colors = np.ones((md.faceCount(), 4), dtype=float)
colors[::2,0] = 0
colors[:,1] = np.linspace(0, 1, colors.shape[0])
md.setFaceColors(colors)
m3 = gl.GLMeshItem(meshdata=md, smooth=False)#, shader='balloon')
w.addItem(m3)

target = gl.MeshData.sphere(4,4,10)
targetMI = gl.GLMeshItem(meshdata = target, drawFaces = True,smooth = False)
w.addItem(targetMI)
while(1):
    targetMI.translate(0.1,0,0)
    app.processEvents()



## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
