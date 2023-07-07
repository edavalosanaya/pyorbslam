import pathlib
import os
import logging

import numpy as np
import cv2
import pyorbslam

logger = logging.getLogger("pyorbslam")

# Constants
GIT_ROOT = pathlib.Path(os.path.abspath(__file__)).parent.parent
SETTING_FILE = GIT_ROOT / 'settings' / "EuRoC.yaml"
EUROC_TEST_DATASET = GIT_ROOT / 'test' / 'data' / 'EuRoC' / 'MH01'

image_filenames, timestamps = pyorbslam.utils.load_images_EuRoC(EUROC_TEST_DATASET)

slam = pyorbslam.MonoSLAM(SETTING_FILE)
assert isinstance(slam, pyorbslam.MonoSLAM)


for i in range(100):
    
    image = cv2.imread(image_filenames[i])

    if type(image) == type(None):
        raise ValueError(f"failed to load image: {image_filenames[i]}")

    state = slam.process(image, timestamps[i])
    pose = np.array([1,2,3])

    if state == pyorbslam.State.OK:
        pose = slam.get_pose_to_target()
        # drawer.plot_trajectory(euroc_slam)
    
    logger.debug(f"{state}")

    cv2.imshow('frame', image)
    cv2.waitKey(1)

slam.shutdown()
