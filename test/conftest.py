import pathlib
import os

import pytest

import pyorbslam

TEST_DIR = pathlib.Path(os.path.abspath(__file__)).parent
GIT_ROOT = TEST_DIR.parent
SETTINGS_DIR = GIT_ROOT / 'settings'
DEFAULT_VOCAB = GIT_ROOT / 'cpp' / 'ORB_SLAM3' / 'Vocabulary' / 'ORBvoc.txt'
EUROC_TEST_DATASET = TEST_DIR / 'data' / 'EuRoC' / 'MH01'

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
