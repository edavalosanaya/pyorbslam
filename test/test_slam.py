import time
import logging
import numpy as np
import imutils
import pickle
import time
import os

import pandas as pd
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