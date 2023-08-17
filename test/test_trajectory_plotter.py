import time
import pickle
import logging

import pandas as pd
import imutils
import cv2
import numpy as np
import pytest
import pyorbslam

from .conftest import TEST_DIR, OUTPUTS_DIR

logger = logging.getLogger("pyorbslam")

EXAMPLE_TRAJECTORY = TEST_DIR / 'data' / 'trajectory.pkl'
MONITOR_PLY = TEST_DIR / 'assets' / 'monitor.ply'
MONITOR_POSE = TEST_DIR / 'assets' / 'tobii_2500.npy'
    
@pytest.fixture
def example_trajectory():
    
    with open(EXAMPLE_TRAJECTORY, 'rb') as f:
        data = pickle.load(f)

    return data


def test_qt_app():
    app = pyorbslam.TDApp()
    app.run()


def test_start_and_stop_app():
    
    drawer = pyorbslam.TrajectoryDrawer()
    time.sleep(2)


def test_plot_line():
    drawer = pyorbslam.TrajectoryDrawer()

    N = 100
    line = np.random.uniform(low=0, high=1, size=(N, 3))
    drawer.plot_line(line)

    drawer.stay()


def test_plot_trajectory(example_trajectory):
    drawer = pyorbslam.TrajectoryDrawer()

    for pose in example_trajectory:
        drawer.plot_trajectory(pose)
        time.sleep(0.05)

    drawer.stay()


def test_image_streaming():
    
    test_video = TEST_DIR/'data'/'scenevideo.mp4'
    assert test_video.exists()

    cap = cv2.VideoCapture(str(test_video), 0)
    drawer = pyorbslam.TrajectoryDrawer()
    
    # for i in range(300):
    i = 0
    while True:

        ret, frame = cap.read()
        logger.debug(f"Frame ID: {i}")
        drawer.plot_image(imutils.resize(frame, width=500))
        i += 1
        time.sleep(1/40)
        # time.sleep(0.5)

def test_trajectory_and_image():
        
    # Load video
    test_video = TEST_DIR/'data'/'scenevideo.mp4'
    assert test_video.exists()
    cap = cv2.VideoCapture(str(test_video), 0)

    # Create drawer
    drawer = pyorbslam.TrajectoryDrawer()

    # Load trajectory
    tobii_trajectory_path = TEST_DIR / 'data' / 'tobii_trajectory_path.csv'
    tobii_trajectory_path.exists()
    pose_data = pd.read_csv(str(tobii_trajectory_path))

    for i in range(len(pose_data)):

        _, frame = cap.read()
        pose = pyorbslam.tools.string_to_numpy(pose_data.iloc[i].pose)

        # Show
        try:
            drawer.plot_trajectory(pose)
            drawer.plot_image(imutils.resize(frame, width=200))
        except Exception as e:
            logger.error(e)

def test_point_cloud_visualization():
    
    drawer = pyorbslam.TrajectoryDrawer()

    N = 100
    for i in range(10):
        pts = np.random.uniform(low=0, high=1, size=(N, 3))
        colors = np.random.uniform(low=0, high=1, size=(N,4))
        drawer.plot_pointcloud('test', pts, colors)
        time.sleep(0.5)

    drawer.stay()

def test_visualize_pose_image_and_pointcloud():
    
    drawer = pyorbslam.TrajectoryDrawer()
   
    # Iterate over the large dataset
    for i in range(10, 500):
        # load
        data = pyorbslam.tools.load_slam_data(i, OUTPUTS_DIR / 'large_dataset')

        # Visualize
        drawer.plot_image(data['image'])
        drawer.plot_trajectory(data['pose'])
        drawer.plot_pointcloud('pc', data['point cloud'])
        time.sleep(0.5)
        # time.sleep(1/60)

    drawer.stay()


def test_plot_gaze_vector(example_trajectory):
    drawer = pyorbslam.TrajectoryDrawer()

    line = np.array([
        [0,0,0],
        [0,0,1]
    ])

    for pose in example_trajectory:

        # Compute gaze vector
        gaze_vector = pyorbslam.tools.apply_rt_to_pts(line, pose)
        
        # Draw
        drawer.plot_line(gaze_vector)
        drawer.plot_trajectory(pose)
        time.sleep(0.05)

    drawer.stay()
