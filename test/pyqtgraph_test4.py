import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow
import numpy as np
import sys

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Create an ImageItem
        image_item = pg.ImageItem()

        # Generate a random image (512x512)
        image_data = np.random.randint(0, 255, size=(512, 512), dtype=np.uint8)

        # Set the image data
        image_item.setImage(image_data)

        # Create a PlotWidget and add the ImageItem
        plot_widget = pg.PlotWidget()
        plot_widget.addItem(image_item)

        # Set the central widget of the main window
        self.setCentralWidget(plot_widget)

if __name__ == '__main__':
    app = QApplication(sys.argv)

    # Create the main window
    mainWindow = MainWindow()
    mainWindow.show()

    # Start the application event loop
    sys.exit(app.exec_())
