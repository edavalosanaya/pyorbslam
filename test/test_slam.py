import time
import logging
import numpy as np
import imutils
import pickle
import time

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

    for i in range(500):
        
        image = cv2.imread(image_filenames[i])

        if type(image) == type(None):
            raise ValueError(f"failed to load image: {image_filenames[i]}")

        state = euroc_slam.process(image, timestamps[i])

        if state == pyorbslam.State.OK:
            pose = euroc_slam.get_pose_to_target()
            # drawer.plot_trajectory(pose)
        
        cv2.imshow('frame', image)
        cv2.waitKey(1)
    
    # Save the information
    with open(TEST_DIR/'data'/'trajectory.pkl', 'wb') as f:
        pickle.dump(euroc_slam.pose_array, f)

        
def test_running_mono_slam_on_tobii(tobii_slam):
   
    test_video = TEST_DIR/'data'/'scenevideo.mp4'
    assert test_video.exists()

    cap = cv2.VideoCapture(str(test_video), 0)
    drawer = pyorbslam.TrajectoryDrawer()

    timestamp = 0
    fps = 1/24
    i = 0

    # for i in range(300):
    while True:

        tic = time.time()
        ret, frame = cap.read()

        timestamp += fps

        if i % 2 == 0: 
            state = tobii_slam.process(frame, timestamp)

            if state == pyorbslam.State.OK:
                pose = tobii_slam.get_pose_to_target()
                drawer.plot_trajectory(pose)
                # logger.debug(f"pose: {pose}")

        toc = time.time()
        i += 1
        drawer.plot_image(imutils.resize(frame, width=500))
        # cv2.imshow('frame', imutils.resize(frame, width=500))
        # cv2.waitKey(1)
       
        # import pdb; pdb.set_trace()
        logger.debug(f"{tobii_slam.get_state()} - {(1/(toc - tic)):.2f}")
