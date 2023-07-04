import pathlib
import sys
import site

# Adding the folder with all the .so files
site_packages_path = pathlib.Path(site.getsitepackages()[0])
# sys.path.append(str(site_packages_path / 'pyorbslam'))


from .__logger import setup
from .trajectory_drawer import TrajectoryDrawer
from .state import State
from .sensor import Sensor
from . import utils
from .slam import MonoSLAM, MonoIMUSLAM, StereoSLAM, StereoIMUSLAM, RgbdSLAM

# Setup logging
setup()

__all__ = [
    'State',
    'Sensor',
    'TrajectoryDrawer',
    'utils',
    'MonoSLAM',
    'MonoIMUSLAM',
    'StereoSLAM',
    'StereoIMUSLAM',
    'RgbdSLAM'
]
