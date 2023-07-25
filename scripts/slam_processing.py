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
ONLINE_MODE = False
N = 5000
P_ID = '220780'
MONITOR_SURFACE_TRI_INDEX = 9
START_IDX = 2000

slam = pyorbslam.MonoSLAM(SETTINGS_DIR / 'Tobii.yaml')
drawer = pyorbslam.TrajectoryDrawer(port=9009)

# Load the video
video_path = GIT_ROOT/'test'/'data'/P_ID/'scenevideo.mp4'
assert video_path.exists()
cap = cv2.VideoCapture(str(video_path))
cap.set(cv2.CAP_PROP_POS_FRAMES, START_IDX)

# Delete for now
if ONLINE_MODE and (OUTPUTS_DIR/P_ID).exists():
    shutil.rmtree(OUTPUTS_DIR /P_ID)

# Resetting the SLAM every time before running on video
slam.reset()
drawer.reset()

# Necessary things to track the progress of SLAM
timestamp = 0
fps = 1/24
i = 0
        
_correction_rt = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, -1, 0, 0],
    [0, 0, 0, 1]
])

# Create the gaze line
line = np.array([
    [0,0,0],
    [0,0,1]
])

correct_rt = np.array([
    [1, 0, 0, 0],
    [0, 0, -1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
])
        
# r = np.deg2rad(np.array([90, 0, 0]))
# R, _ = cv2.Rodrigues(r.astype(np.float32))
# correct_rt = np.eye(4)
# correct_rt[:3, :3] = R

# Create the monitor
# 3D object information
t = np.array([0.35, 0.05, 0.15])
r = np.deg2rad(np.array([-20, 60, 20]))
R, _ = cv2.Rodrigues(r.astype(np.float32))
rt = np.eye(4)
rt[:3, :3] = R
rt[:3, 3] = t

# Load 3D model
monitor: trimesh.Trimesh = trimesh.load_mesh(str(MONITOR_PLY))
monitor.vertices -= monitor.centroid
monitor = monitor.apply_scale(0.012)
monitor = monitor.apply_transform(rt)

# Draw monitor
drawer.add_mesh('monitor', monitor, color=(0.0, 0.0, 1.0, 1.0))
drawer.update_mesh('monitor', monitor, color=(0.0, 0.0, 1.0, 1.0))

# Stuff to perform intersection information
int_monitor = monitor.copy().apply_transform(_correction_rt)

# Data Container
data = {}

# for i in range(N):
for i in tqdm.tqdm(range(START_IDX, int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))):

    # Loading the frame
    tic = time.time()
    ret, frame = cap.read()
       
    # Data Container
    # data = {'pose': np.empty((0,3)), 'point cloud': np.empty((0,3))}

    if ONLINE_MODE:
        # Process the frame
        state = slam.process(frame, timestamp)

        # Extract the data
        if state == pyorbslam.State.OK:
            data['pose'] = slam.get_pose_to_target()
            data['point cloud'] = slam.get_current_map_points()
        
            # Save data
            pyorbslam.tools.record_slam_data(i, slam, OUTPUTS_DIR / P_ID, save_rate=10)

    else:
    
        # Load data
        data = pyorbslam.tools.load_slam_data(i, OUTPUTS_DIR / P_ID)

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
            r = data['pose'].copy()
            r[:3, 3] = 0

            # Create the ray information
            ray_origin = pyorbslam.tools.apply_rt_to_pts(
                pyorbslam.tools.apply_rt_to_pts(line, data['pose']),
                _correction_rt
            )[0]
            ray_direction = pyorbslam.tools.apply_rt_to_pts(
                pyorbslam.tools.apply_rt_to_pts(line, r),
                _correction_rt
            )[1]

            # Find intersection
            locations, index_ray, index_tri = int_monitor.ray.intersects_location(
                ray_origins=np.expand_dims(ray_origin, axis=0), 
                ray_directions=np.expand_dims(ray_direction, axis=0)
            )

            # If any, process
            if locations.any():

                # Select the correct face
                face_ids = np.where(index_tri == MONITOR_SURFACE_TRI_INDEX)
                if len(face_ids) and face_ids[0].any():

                    # Get the information about the intersection if it intersects
                    # the correct face
                    face_id = face_ids[0][0]
                    intersection_point = locations[face_id] - int_monitor.centroid
                    norm_coord = (intersection_point + 0.5*int_monitor.extents) / int_monitor.extents
                    r_x, r_y = norm_coord[:2]
                    logger.debug(f"Intersection hit at ({r_x:.3f},{r_y:.3f})!")

                    # Draw
                    screen_fix = (int(r_x*w), int(r_y*h))
                    screen = cv2.circle(screen, screen_fix, 10, (0,0,255), 3)
                    drawer.plot_pointcloud('inter', np.expand_dims(locations[face_id], axis=0), colors=(1.0, 0.0, 0.0, 1.0))
                    time.sleep(0.01)

            ray_visualize = trimesh.load_path(np.hstack((
                ray_origin,
                ray_origin + ray_direction
            )).reshape((-1,3)))
            
            scene = trimesh.Scene([
                int_monitor,
                ray_visualize,
                # trimesh.points.PointCloud(locations)
            ])
            
            # Define the axis lines
            # We will create a set of line segments representing the x, y, and z axes
            # Each line segment is defined by two points: [p1, p2]
            # We are creating three line segments, each of length 1, along each of the axes
            # X-axis: red, Y-axis: green, Z-axis: blue
            axes_points = np.array([
                [[0, 0, 0], [1, 0, 0]],  # X-axis
                [[0, 0, 0], [0, 1, 0]],  # Y-axis
                [[0, 0, 0], [0, 0, 1]],  # Z-axis
            ])
            for points in axes_points:
                # Create a Path3D object representing the axis
                axis = trimesh.load_path(points.reshape((-1,3)))
                axis.color = points[1]
                # Add the axis to the scene
                scene.add_geometry(axis)

            scene.show()

        # Visualize
        drawer.plot_line(gaze_vector, color=(1.0, 0.0, 0.0, 1.0))
        drawer.plot_image(imutils.resize(np.concatenate([draw_frame, screen], axis=0), width=500))
        drawer.plot_trajectory(data['pose'])
        if 'point cloud' in data:
            # logger.debug(data['point cloud'].shape)
            drawer.plot_pointcloud('pc', data['point cloud'])

    # Update
    i += 1
    timestamp += fps
