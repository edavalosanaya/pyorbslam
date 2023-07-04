from enum import Enum
import numpy as np

class Sensor(Enum):
    """
    This class is used to set the type of the sensor
        Values:
            MONOCULAR
            STEREO
            MONOCULAR_IMU
            STEREO_IMU
            RGBD
    """

    MONOCULAR = 1
    STEREO = 2
    MONOCULAR_IMU = 3
    STEREO_IMU = 4
    RGBD = 5


