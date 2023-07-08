import numpy as np
import vispy.scene
from vispy.scene import visuals
from vispy import app
import glob
import time

pclds = glob.glob("path/to/npz/files/*.npz")
canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)

view = canvas.central_widget.add_view()

scatter = visuals.Markers()
view.camera = 'arcball'  # or try 'arcball'
view.add(scatter)

axis = visuals.XYZAxis(parent=view.scene)
N = 100
pts = np.random.uniform(low=0, high=1, size=(N,3))

def update(event):
    global start

    # print('time taken per frame', time.time() - start)
    # for pcld in pclds:
        # pcld = np.load(pcld)
        # pcld = pcld['Pts_mtx']

        # pos = np.random.normal(size=(500000, 3), scale=0.2)
    colr = np.random.uniform(low=0, high=1, size=(N,3))

    scatter.set_data(pts, edge_color=(0.5, 0.1, 0, .5), face_color=colr, size=5)
    canvas.update()

        # add a colored 3D axis for orientation
        # start = time.time()

timer = app.Timer()
start = time.time()
timer.connect(update)
timer.start(0.05)


if __name__ == '__main__':
	import sys
	if sys.flags.interactive != 1:
		vispy.app.run()
