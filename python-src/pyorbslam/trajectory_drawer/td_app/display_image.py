
import logging
from typing import Dict, Any, Literal

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

logger = logging.getLogger('pyorbslam')

from ..data_chunk import DataChunk

class DisplayImage(pg.PlotWidget):

    visuals: Dict[str, Any] # Dict[str, Item]

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        # Create an ImageItem
        image_item = pg.ImageItem()

        # Generate a random image (512x512)
        image_data = np.random.randint(0, 255, size=(512, 512), dtype=np.uint8)

        # Set the image data
        image_item.setImage(image_data)

        self.addItem(image_item)

    def __str__(self):
        return "<DisplayImage>"

    def __repr__(self):
        return str(self)
