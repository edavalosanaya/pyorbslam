import pyorbslam

from .conftest import OUTPUTS_DIR

def test_record_slam_data(slam_in_okay):

    # Record
    pyorbslam.tools.record_slam_data(0, slam_in_okay, OUTPUTS_DIR / 'test_record')

    # And load
    data = pyorbslam.tools.load_slam_data(0, OUTPUTS_DIR / 'test_record') 
