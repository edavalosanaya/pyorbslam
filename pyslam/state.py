from enum import Enum
import numpy as np
import importlib
import yaml

class State(Enum):
    """This class is used to get the actual state of the tracking
    Values:
        OK
        LOST
        NOT_INITIALIZED
    """

    OK = 1
    LOST = 2
    NOT_INITIALIZED = 3
    SYSTEM_NOT_READY = 4

