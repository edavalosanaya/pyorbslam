import sys
import os
import logging

import pytest
import cv2
import pyorbslam

logger = logging.getLogger("pyorbslam")

from .conftest import DEFAULT_VOCAB, SETTINGS_DIR, EUROC_TEST_DATASET

@pytest.fixture(scope="module")
def slam_in_okay():
    slam = pyorbslam.MonoSLAM(SETTINGS_DIR / 'EuRoC.yaml')
    
    image_filenames, timestamps = pyorbslam.utils.load_images_EuRoC(EUROC_TEST_DATASET)
    
    for idx in range(min(len(image_filenames), 10)):
        image = cv2.imread(image_filenames[idx], cv2.IMREAD_UNCHANGED)
        tframe = timestamps[idx]

        if image is None:
            print("failed to load image at {0}".format(image_filenames[idx]))
            raise RuntimeError()

        success = slam.process(image, tframe)
        logger.debug(f"Success: {success}")

    logger.debug("Passing off SLAM")
    yield slam
    slam.shutdown()

# @pytest.mark.skip(reason="SEGFAULT")
def test_get_current_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_current_points()}")

def test_get_frame_pose(slam_in_okay): # PASSING
    logger.debug(f"{slam_in_okay.slam.get_frame_pose()}")

# @pytest.mark.skip(reason="SEGFAULT")
def test_get_camera_matrix(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_camera_matrix()}")

# @pytest.mark.skip(reason="not SEGFAULT")
def test_get_dist_coef(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_dist_coef()}")

def test_get_keyframe_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_keyframe_points()}")
    
# @pytest.mark.skip(reason="SEGFAULT")
def test_get_trajectory_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_trajectory_points()}")

def test_get_tracked_mappoints(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_tracked_mappoints()}")

def test_get_num_features(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_num_features()}")

# @pytest.mark.skip(reason="SEGFAULT")
def test_get_num_matched_features(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_num_matched_features()}")
