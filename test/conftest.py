import pathlib
import os
import logging

import pytest
import cv2

import pyorbslam

logger = logging.getLogger("pyorbslam")

TEST_DIR = pathlib.Path(os.path.abspath(__file__)).parent
GIT_ROOT = TEST_DIR.parent
OUTPUTS_DIR = TEST_DIR / 'outputs'
SETTINGS_DIR = GIT_ROOT / 'settings'
DEFAULT_VOCAB = GIT_ROOT / 'cpp' / 'ORB_SLAM3' / 'Vocabulary' / 'ORBvoc.txt'
EUROC_TEST_DATASET = TEST_DIR / 'data' / 'EuRoC' / 'MH01'

@pytest.fixture(scope="session")
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
