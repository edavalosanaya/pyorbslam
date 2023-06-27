
import pyslam

from .conftest import SETTINGS_DIR

# Constants
# SETTING_FILE = SETTINGS_DIR / "OrbSlam3_TUM_freiburg3.yaml"
# SETTING_FILE = SETTINGS_DIR / "EuRoC_ViconRoom2.yaml"
SETTING_FILE = SETTINGS_DIR / "EuRoC.yaml"
assert SETTING_FILE.exists()

def test_mono_slam():
    slam = pyslam.MonoSLAM(SETTING_FILE)
    assert isinstance(slam, pyslam.MonoSLAM)
