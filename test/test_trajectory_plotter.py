import time
import pickle
import logging

import numpy as np
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


def test_qt_app():
    app = pyorbslam.TDApp()
    app.run()


def test_start_and_stop_app():
    
    drawer = pyorbslam.TrajectoryDrawer()
    time.sleep(2)


def test_plot_line(example_trajectory):
    drawer = pyorbslam.TrajectoryDrawer()

    N = 10
    line = np.random.uniform(low=0, high=1, size=(N, 3))
    drawer.plot_path(line)

    drawer.stay()


def test_plot_trajectory(example_trajectory):
    drawer = pyorbslam.TrajectoryDrawer()

    for pose in example_trajectory:
        drawer.plot_trajectory(pose)
        time.sleep(0.05)

    drawer.stay()
