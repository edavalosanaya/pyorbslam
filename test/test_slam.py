import time
import logging
import numpy as np

import pytest
import cv2

import pyorbslam

from .conftest import SETTINGS_DIR, TEST_DIR, EUROC_TEST_DATASET

logger = logging.getLogger("pyorbslam")

# Constants
EUROC_TEST_DATASET = TEST_DIR / 'data' / 'EuRoC' / 'MH01'

def test_mono_slam():
    slam = pyorbslam.MonoSLAM(SETTINGS_DIR / 'EuRoC_ViconRoom2.yaml')
    assert isinstance(slam, pyorbslam.MonoSLAM)
    slam.shutdown()


def test_mono_slam_euroc(euroc_slam):
    image_filenames, timestamps = pyorbslam.utils.load_images_EuRoC(EUROC_TEST_DATASET)
    # drawer = pyorbslam.TrajectoryDrawer()
    # fig = drawer.get_figure()
    # fig.show()

    for i in range(100):
        
        image = cv2.imread(image_filenames[i])

        if type(image) == type(None):
            raise ValueError(f"failed to load image: {image_filenames[i]}")

        state = euroc_slam.process(image, timestamps[i])
        pose = np.array([1,2,3])

        if state == pyorbslam.State.OK:
            pose = euroc_slam.get_pose_to_target()
            # drawer.plot_trajectory(euroc_slam)
        
        logger.debug(f"{state}")

        cv2.imshow('frame', image)
        cv2.waitKey(1)

        

def test_running_mono_slam_on_tobii(tobii_slam):
   
    test_video = TEST_DIR/'data'/'scenevideo.mp4'
    assert test_video.exists()

    cap = cv2.VideoCapture(str(test_video), 0)
    drawer = pyorbslam.TrajectoryDrawer()
    drawer.get_figure()

    timestamp = 0
    fps = 1/24

    for i in range(100):

        ret, frame = cap.read()

        state = tobii_slam.process(frame, timestamp)
        timestamp += fps

        logger.debug(tobii_slam.get_state())

        if state == pyorbslam.State.OK:
            # pose = tobii_slam.get_pose_to_target()
            drawer.plot_trajectory(tobii_slam)
            # logger.debug(f"pose: {pose}")

        cv2.imshow('frame', frame)
        cv2.waitKey(1)
