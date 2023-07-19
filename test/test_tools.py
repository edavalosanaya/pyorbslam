import cv2
import pyorbslam

from .conftest import OUTPUTS_DIR, EUROC_TEST_DATASET

def test_record_slam_data(slam_in_okay):

    # Record
    pyorbslam.tools.record_slam_data(0, slam_in_okay, OUTPUTS_DIR / 'test_record')

    # And load
    data = pyorbslam.tools.load_slam_data(0, OUTPUTS_DIR / 'test_record')

def test_record_500_slam_data(slam_in_okay):

    image_filenames, timestamps = pyorbslam.utils.load_images_EuRoC(EUROC_TEST_DATASET)

    for idx in range(10, 510):
        image = cv2.imread(image_filenames[idx], cv2.IMREAD_UNCHANGED)
        tframe = timestamps[idx]

        if image is None:
            print("failed to load image at {0}".format(image_filenames[idx]))
            raise RuntimeError()

        success = slam_in_okay.process(image, tframe)

        # Save data
        pyorbslam.tools.record_slam_data(idx, slam_in_okay, OUTPUTS_DIR / 'large_dataset')
