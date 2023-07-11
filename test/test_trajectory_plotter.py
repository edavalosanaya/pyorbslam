import time
import pickle
import logging

import imutils
import cv2
import numpy as np
import pytest
import pyorbslam

from .conftest import TEST_DIR

logger = logging.getLogger("pyorbslam")

EXAMPLE_TRAJECTORY = TEST_DIR / 'data' / 'trajectory.pkl'

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
