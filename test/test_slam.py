
import pyslam

from .conftest import TEST_DIR

# Constants
SETTING_FILE = TEST_DIR / "assets" / "settings_EuRoC.yaml"

def test_slam_initialization():
    slam = pyslam.System(str(SETTING_FILE), pyslam.Sensor.MONOCULAR)
