import pathlib
import os

import pyslam

SETTING_FILE = pathlib.Path(os.path.abspath(__file__)).parent.parent / 'settings' / "EuRoC.yaml"

slam = pyslam.MonoSLAM(SETTING_FILE)
assert isinstance(slam, pyslam.MonoSLAM)
slam.shutdown()
