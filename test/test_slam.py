import time
import logging

import pytest
import cv2

import pyslam

from .conftest import SETTINGS_DIR, TEST_DIR

logger = logging.getLogger("pyslam")

# Constants
# SETTING_FILE = SETTINGS_DIR / "OrbSlam3_TUM_freiburg3.yaml"
SETTING_FILE = SETTINGS_DIR / "EuRoC_ViconRoom2.yaml"
# SETTING_FILE = SETTINGS_DIR / "EuRoC.yaml"
assert SETTING_FILE.exists()


@pytest.fixture
def tobii_slam():
    slam = pyslam.MonoSLAM(SETTINGS_DIR / 'Tobii.yaml')
    yield slam
    slam.shutdown()


def test_mono_slam():
    slam = pyslam.MonoSLAM(SETTING_FILE)
    assert isinstance(slam, pyslam.MonoSLAM)
    slam.shutdown()


def test_running_mono_slam(tobii_slam):
    
    cap = cv2.VideoCapture(str(TEST_DIR/'data'/'scenevideo.mp4'), 0)

    timestamp = 0
    fps = 1/24

    for i in range(100):

        ret, frame = cap.read()

        state = tobii_slam.process(frame, timestamp)
        timestamp += fps

        if state == pyslam.State.OK:
            logger.debug(i)

        cv2.imshow('frame', frame)
        cv2.waitKey(1)
