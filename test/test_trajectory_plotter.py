import time
import pickle
import logging

import trimesh
import pandas as pd
import imutils
import cv2
import numpy as np
import pytest
import pyorbslam

from .conftest import TEST_DIR, EUROC_TEST_DATASET

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


def test_plot_line(example_trajectory):
    drawer = pyorbslam.TrajectoryDrawer()

    N = 100
    line = np.random.uniform(low=0, high=1, size=(N, 3))
    drawer.plot_path(line)

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
        
    rt = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])
    # monitor_pose = np.array([
    #     [ 9.97679500e-01, -2.47593077e-02,  6.34239133e-02, 1.47544021e+01],
    #     [ 2.71596339e-02,  9.98936183e-01, -3.72673573e-02,-4.87549905e+00],
    #     [-6.24337279e-02,  3.89034486e-02,  9.97290605e-01,3.14412418e+01],
    #     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,1.00000000e+00]]
    # )
    monitor_pose = np.array([
        [ 9.97679500e-01, -2.47593077e-02,  6.34239133e-02, 0.147],
        [ 2.71596339e-02,  9.98936183e-01, -3.72673573e-02, -.4],
        [-6.24337279e-02,  3.89034486e-02,  9.97290605e-01, 3],
        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,1.00000000e+00]]
    )

    # Load video
    test_video = TEST_DIR/'data'/'scenevideo.mp4'
    assert test_video.exists()
    cap = cv2.VideoCapture(str(test_video), 0)

    # Load trajectory
    tobii_trajectory_path = TEST_DIR / 'data' / 'tobii_trajectory_path.csv'
    tobii_trajectory_path.exists()
    pose_data = pd.read_csv(str(tobii_trajectory_path))

    # Load 3D model
    monitor: trimesh.Trimesh = trimesh.load_mesh(str(MONITOR_PLY))
    monitor = monitor.apply_scale(0.05)
    # monitor_pose = np.load(str(MONITOR_POSE))
    monitor = monitor.apply_transform(monitor_pose)
    monitor = monitor.apply_transform(rt)

    # Create drawer
    drawer = pyorbslam.TrajectoryDrawer()

    # Configure monitor
    drawer.add_mesh('monitor', monitor, color=(0,1,0,1))

    i = 0

    def string_to_numpy(string):
        string = string.replace('[', '').replace(']', '').replace('\n', '')
        array = np.fromstring(string, sep=' ')
        array = array.reshape((4, 4))
        return array

    # for i in range(500):
    for j in range(len(pose_data)):

        # Get the data
        ret, frame = cap.read()
        pose = string_to_numpy(pose_data.iloc[i].pose)

        # Show
        try:
            drawer.plot_trajectory(pose)
            drawer.plot_image(imutils.resize(frame, width=200))
        except Exception as e:
            logger.error(e)
        
        # Update
        i += 1
        logger.debug(f"i: {i}") # 2,500 is stable!
