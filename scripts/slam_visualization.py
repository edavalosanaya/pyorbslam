import pathlib
import time
import os

import trimesh
import pandas as pd
import pyorbslam
import cv2
import numpy as np
import imutils

# And let's get the path to the GIT directory and other import directories
GIT_ROOT = pathlib.Path(os.path.abspath(__file__)).parent.parent
SETTINGS_DIR = GIT_ROOT / 'settings'
MONITOR_PLY = GIT_ROOT / 'notebooks' / 'data' / 'monitor.ply'
OUTPUTS_DIR = GIT_ROOT / 'scripts' / 'outputs'

drawer = pyorbslam.TrajectoryDrawer(port=9000)

# Add monitor
correct_rt = np.array([
    [1, 0, 0, 0],
    [0, 0, -1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
])
t = np.array([0.9, -0.15, 1.5])
r = np.array([-0.1, 0.5, 0.18])
R, _ = cv2.Rodrigues(r.astype(np.float32))
rt = np.eye(4)
rt[:3, :3] = R
rt[:3, 3] = t

# monitor_pose = np.array([
#     [ 9.97679500e-01, -2.47593077e-02,  6.34239133e-02, 0.235], #left-right
#     [ 2.71596339e-02,  9.98936183e-01, -3.72673573e-02, 0.04875], #+1 -> down
#     [-6.24337279e-02,  3.89034486e-02,  9.97290605e-01, 0.6],
#     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,1.00000000e+00]]
# )

# Load 3D model
monitor: trimesh.Trimesh = trimesh.load_mesh(str(MONITOR_PLY))
monitor = monitor.apply_scale(0.03)
monitor = monitor.apply_transform(rt)
monitor = monitor.apply_transform(correct_rt)

# Configure monitor
drawer.add_mesh('monitor', monitor, color=(0.0,1.0,0.0,1.0))
drawer.update_mesh('monitor', monitor, color=(0.0,1.0,0.0,1.0))

# Necessary things to track the progress of SLAM
i = 0

while True:

    # Load data
    data = pyorbslam.tools.load_slam_data(i, OUTPUTS_DIR / 'tobii')

    # Draw the gaze    
    line = np.array([
        [0,0,0],
        [0,0,2]
    ])

    if data:

        # Compute the gaze vector
        pose = data['pose']
        gaze_vector = pyorbslam.tools.apply_rt_to_pts(line, pose)

        # Draw fixation and change to COLOR image
        img = cv2.cvtColor(data['image'], cv2.COLOR_GRAY2RGB)
        h, w = img.shape[:2]
        fix = (int(w/2), int(h/2))
        img = cv2.circle(img, fix, 10, (0,255,0), 3)

        drawer.plot_line(gaze_vector, color=(1.0, 0.0, 0.0, 1.0), width=5)
        drawer.plot_image(img)
        drawer.plot_trajectory(data['pose'])
        drawer.plot_pointcloud('pc', data['point cloud'])
        time.sleep(1/40)

    else:
        time.sleep(1/40)

    # Update
    i += 1
