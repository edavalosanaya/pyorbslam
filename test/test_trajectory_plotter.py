import time
import pickle
import logging

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

def test_plot_trajectory(example_trajectory):
    
    drawer = pyorbslam.TrajectoryDrawer()
    logger.debug(f"Trajectory: {example_trajectory}")

    for pose in example_trajectory:
        drawer.plot_trajectory(pose)