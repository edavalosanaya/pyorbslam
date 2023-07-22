import pathlib
import time
import os

import tqdm
import trimesh
import pandas as pd
import pyorbslam
import cv2
import numpy as np
import imutils

# And let's get the path to the GIT directory and other import directories
# GIT_ROOT = pathlib.Path(os.path.abspath('')).parent
GIT_ROOT = pathlib.Path(os.path.abspath(__file__)).parent.parent
SETTINGS_DIR = GIT_ROOT / 'settings'
MONITOR_PLY = GIT_ROOT / 'notebooks' / 'data' / 'monitor.ply'
OUTPUTS_DIR = GIT_ROOT / 'scripts' / 'outputs'
N = 1000

slam = pyorbslam.MonoSLAM(SETTINGS_DIR / 'Tobii.yaml')
drawer = pyorbslam.TrajectoryDrawer(port=9000)
# map = pyorbslam.Map()

# Load the video
video_path = GIT_ROOT/'test'/'data'/'scenevideo.mp4'
assert video_path.exists()
cap = cv2.VideoCapture(str(video_path))

# Resetting the SLAM every time before running on video
slam.reset()
drawer.reset()

# Necessary things to track the progress of SLAM
timestamp = 0
fps = 1/24
i = 0

# for i in range(N):
for i in tqdm.tqdm(range(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))):

    # Loading the frame
    tic = time.time()
    ret, frame = cap.read()

    # Process the frame
    state = slam.process(frame, timestamp)

    # Only if things work out should we visualize the state
    if state == pyorbslam.State.OK:
        pose = slam.get_pose_to_target()
        drawer.plot_trajectory(pose)

        # Visualize map
        # pc = map.step(slam.get_point_cloud())
        # pc = slam.get_point_cloud()
        pc = slam.get_current_map_points()
        drawer.plot_pointcloud('pc', pc)
        # drawer.plot_pointcloud('pc', slam.get_point_cloud())
        # drawer.plot_pointcloud('pc', slam.get_point_cloud())
        # pyorbslam.tools.record_slam_data(i, slam, OUTPUTS_DIR / 'tobii')

    # Update
    i += 1
    timestamp += fps

    # Show
    drawer.plot_image(imutils.resize(frame, width=200))
