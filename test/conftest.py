import pathlib
import os

TEST_DIR = pathlib.Path(os.path.abspath(__file__)).parent
GIT_ROOT = TEST_DIR.parent
SETTINGS_DIR = GIT_ROOT / 'settings'
DEFAULT_VOCAB = GIT_ROOT / 'cpp' / 'ORB_SLAM3' / 'Vocabulary' / 'ORBvoc.txt'
