
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

        # Configuration
        self.setAspectLocked(True)
        self.hideAxis('left')
        self.hideAxis('bottom')

        # Create an ImageItem
        self.image_item = pg.ImageItem()

        # Generate a random image (512x512)
        image_data = np.random.randint(0, 255, size=(512, 512, 3), dtype=np.uint8)

        # Set the image data
        self.image_item.setImage(image_data)
        self.addItem(self.image_item)

    def __str__(self):
        return "<DisplayImage>"

    def __repr__(self):
        return str(self)

    def update_image(self, image: np.ndarray):
        corrected_image = np.rot90(image, k=3)[:,:,::-1]
        self.image_item.clear()
        self.image_item.setImage(corrected_image.astype(np.uint8))
