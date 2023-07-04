import time
import logging

import pytest
import cv2

import pyorbslam

from .conftest import SETTINGS_DIR, TEST_DIR

logger = logging.getLogger("pyorbslam")

# Constants
# SETTING_FILE = SETTINGS_DIR / "OrbSlam3_TUM_freiburg3.yaml"
SETTING_FILE = SETTINGS_DIR / "EuRoC_ViconRoom2.yaml"
# SETTING_FILE = SETTINGS_DIR / "EuRoC.yaml"
assert SETTING_FILE.exists()


@pytest.fixture
def tobii_slam():
    slam = pyorbslam.MonoSLAM(SETTINGS_DIR / 'Tobii.yaml')
    yield slam
    slam.shutdown()


@pytest.fixture
def euroc_slam():
    slam = pyorbslam.MonoSLAM(SETTINGS_DIR / 'EuRoC.yaml')
    yield slam
    slam.shutdown()


def test_mono_slam():
    slam = pyorbslam.MonoSLAM(SETTING_FILE)
    assert isinstance(slam, pyorbslam.MonoSLAM)
    slam.shutdown()


def test_mono_slam_euroc(euroc_slam):
    image_filenames, timestamps = pyorbslam.utils.load_images_EuRoC("/home/nicole/Datasets/EuRoC/MH01")
    drawer = pyorbslam.TrajectoryDrawer()
    fig = drawer.get_figure()
    fig.show()

    for i in range(100):
        
        image = cv2.imread(image_filenames[i])

        if type(image) == type(None):
            raise ValueError(f"failed to load image: {image_filenames[i]}")

        state = euroc_slam.process(image, timestamps[i])

        if state == pyorbslam.State.OK:
            pose = euroc_slam.get_pose_to_target()
            # drawer.plot_trajectory(euroc_slam)
            # logger.debug(f"{state}, {pose}")

        cv2.imshow('frame', image)
        cv2.waitKey(1)

        

def test_running_mono_slam_on_tobii(tobii_slam):
    
    cap = cv2.VideoCapture(str(TEST_DIR/'data'/'scenevideo.mp4'), 0)
    drawer = pyorbslam.TrajectoryDrawer()
    drawer.get_figure()

    timestamp = 0
    fps = 1/24

    for i in range(100):

        ret, frame = cap.read()

        state = tobii_slam.process(frame, timestamp)
        timestamp += fps

        logger.debug(tobii_slam.get_state())

        # if state == pyorbslam.State.OK:

        # cv2.imshow('frame', frame)
        # cv2.waitKey(1)
