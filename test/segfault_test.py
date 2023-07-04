import pathlib
import os

import pyorbslam

SETTING_FILE = pathlib.Path(os.path.abspath(__file__)).parent.parent / 'settings' / "EuRoC.yaml"

slam = pyorbslam.MonoSLAM(SETTING_FILE)
assert isinstance(slam, pyorbslam.MonoSLAM)
slam.shutdown()
