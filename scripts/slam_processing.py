import pathlib
import time
import os
import logging
import shutil

import tqdm
import trimesh
import pandas as pd
import pyorbslam
import cv2
import numpy as np
import imutils

logger = logging.getLogger('pyorbslam')

# And let's get the path to the GIT directory and other import directories
# GIT_ROOT = pathlib.Path(os.path.abspath('')).parent
GIT_ROOT = pathlib.Path(os.path.abspath(__file__)).parent.parent
SETTINGS_DIR = GIT_ROOT / 'settings'
MONITOR_PLY = GIT_ROOT / 'notebooks' / 'data' / 'monitor.ply'
OUTPUTS_DIR = GIT_ROOT / 'scripts' / 'outputs'
ONLINE_MODE = True
N = 5000

slam = pyorbslam.MonoSLAM(SETTINGS_DIR / 'Tobii.yaml')
drawer = pyorbslam.TrajectoryDrawer(port=9009)

# Load the video
video_path = GIT_ROOT/'test'/'data'/'220780'/'scenevideo.mp4'
assert video_path.exists()
cap = cv2.VideoCapture(str(video_path))

# Delete for now
if ONLINE_MODE and (OUTPUTS_DIR/'220780').exists():
    shutil.rmtree(OUTPUTS_DIR / '220780')

# Resetting the SLAM every time before running on video
slam.reset()
drawer.reset()

# Necessary things to track the progress of SLAM
timestamp = 0
fps = 1/24
i = 0

# Create the gaze line
line = np.array([
    [0,0,0],
    [0,0,5]
])

# Create the monitor
# 3D object information
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

# Load 3D model
monitor: trimesh.Trimesh = trimesh.load_mesh(str(MONITOR_PLY))
monitor = monitor.apply_scale(0.03)
monitor = monitor.apply_transform(rt)
monitor = monitor.apply_transform(correct_rt)

# Draw monitor
drawer.add_mesh('monitor', monitor, color=(0.0, 1.0, 0.0, 1.0))
drawer.update_mesh('monitor', monitor, color=(0.0, 1.0, 0.0, 1.0))

# for i in range(N):
for i in tqdm.tqdm(range(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))):

    # Loading the frame
    tic = time.time()
    ret, frame = cap.read()
       
    # Data Container
    # data = {'pose': np.empty((0,3)), 'point cloud': np.empty((0,3))}
    data = {}

    if ONLINE_MODE:
        # Process the frame
        state = slam.process(frame, timestamp)

        # Extract the data
        if state == pyorbslam.State.OK:
            data['pose'] = slam.get_pose_to_target()
            data['point cloud'] = slam.get_current_map_points()
        
            # Save data
            # pyorbslam.tools.record_slam_data(i, slam, OUTPUTS_DIR / '220113', save_rate=10)

    else:
    
        # Load data
        data = pyorbslam.tools.load_slam_data(i, OUTPUTS_DIR / '220780')

    # Only if things work out should we visualize the state
    if data:

        # Gaze
        gaze_vector = pyorbslam.tools.apply_rt_to_pts(line, data['pose'])

        # Draw gaze on the image
        h, w = frame.shape[:2]
        fix = (int(w/2),int(h/2))
        draw_frame = cv2.circle(frame, fix, 10, (0,0,255), 3)
        
        # Create a screen point
        screen = np.ones_like(draw_frame) * 255

        if not ONLINE_MODE:
            # Determine if gaze collision with monitor
            gaze_vector_world = drawer._correct_pts(gaze_vector)
            monitor_world = monitor.apply_transform(drawer._correction_rt)
            locations, index_ray, index_tri = monitor_world.ray.intersects_location(
                ray_origins=np.expand_dims(gaze_vector_world[0], axis=0), 
                ray_directions=np.expand_dims(gaze_vector_world[1], axis=0)
            )
            if locations.any():
                intersection_point = locations[0] - monitor.centroid
                norm_coord = (intersection_point + 0.5*monitor.extents) / monitor.extents
                r_x, r_y = norm_coord[:2]
                screen_fix = (int(r_x*w), int(r_y*h))
                screen = cv2.circle(screen, screen_fix, 10, (0,0,255), 3)

        # Visualize
        # drawer.plot_line(gaze_vector, color=(1.0, 0.0, 0.0, 1.0))
        # drawer.plot_image(imutils.resize(np.concatenate([draw_frame, screen], axis=0), width=500))
        # drawer.plot_trajectory(data['pose'])
        # if 'point cloud' in data:
        #     drawer.plot_pointcloud('pc', data['point cloud'])

    # Update
    i += 1
    timestamp += fps
