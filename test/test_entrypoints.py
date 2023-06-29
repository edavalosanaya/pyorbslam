import sys
import os
import logging

import pytest
import cv2
import pyslam
import orbslam3

logger = logging.getLogger("pyslam")

# DONE: .def("initialize", &ORBSlamPython::initialize)
# .def("load_and_process_mono", &ORBSlamPython::loadAndProcessMono)
# DONE: .def("process_image_mono", &ORBSlamPython::processMono)
# .def("load_and_process_imu_mono", &ORBSlamPython::loadAndProcessImuMono)
# .def("process_image_imu_mono", &ORBSlamPython::processImuMono)
# .def("load_and_process_stereo", &ORBSlamPython::loadAndProcessStereo)
# .def("process_image_stereo", &ORBSlamPython::processStereo)
# .def("load_and_process_imu_stereo", &ORBSlamPython::loadAndProcessImuStereo)
# .def("process_image_imu_stereo", &ORBSlamPython::processImuStereo)
# .def("load_and_process_rgbd", &ORBSlamPython::loadAndProcessRGBD)
# .def("process_image_rgbd", &ORBSlamPython::processRGBD)
# .def("shutdown", &ORBSlamPython::shutdown)
# .def("is_running", &ORBSlamPython::isRunning)
# .def("reset", &ORBSlamPython::reset)
# .def("activateSLAM", &ORBSlamPython::activateSLAMTraking)
# .def("deactivateSLAM", &ORBSlamPython::deactivateSLAMTraking)
# .def("get_current_points", &ORBSlamPython::getCurrentPoints)
# .def("get_frame_pose", &ORBSlamPython::getFramePose)
# .def("get_camera_matrix", &ORBSlamPython::getCameraMatrix)
# .def("get_dist_coef", &ORBSlamPython::getDistCoeff)
# .def("set_mode", &ORBSlamPython::setMode)
# .def("set_use_viewer", &ORBSlamPython::setUseViewer)
# .def("get_keyframe_points", &ORBSlamPython::getKeyframePoints)
# .def("get_trajectory_points", &ORBSlamPython::getTrajectoryPoints)
# .def("get_tracked_mappoints", &ORBSlamPython::getTrackedMappoints)
# .def("get_tracking_state", &ORBSlamPython::getTrackingState)
# .def("get_num_features", &ORBSlamPython::getNumFeatures)
# .def("get_num_matched_features", &ORBSlamPython::getNumMatches)
# .def("save_settings", &ORBSlamPython::saveSettings)
# .def("load_settings", &ORBSlamPython::loadSettings)
# .def("save_settings_file", &ORBSlamPython::saveSettingsFile)
# .staticmethod("save_settings_file")
# .def("load_settings_file", &ORBSlamPython::loadSettingsFile)


from .conftest import DEFAULT_VOCAB, SETTINGS_DIR

@pytest.fixture(scope="module")
def slam_in_okay():
    slam = orbslam3.System(str(DEFAULT_VOCAB), str(SETTINGS_DIR / 'EuRoC.yaml'), orbslam3.Sensor.MONOCULAR)
    slam.set_use_viewer(False)
    slam.initialize()
    
    image_filenames, timestamps = pyslam.utils.load_images_EuRoC("/home/nicole/Datasets/EuRoC/MH01")
    
    for idx in range(min(len(image_filenames), 10)):
        image = cv2.imread(image_filenames[idx], cv2.IMREAD_UNCHANGED)
        tframe = timestamps[idx]

        if image is None:
            print("failed to load image at {0}".format(image_filenames[idx]))
            raise RuntimeError()

        success = slam.process_image_mono(image, tframe, str(image_filenames[idx]))
        logger.debug(f"Success: {success}")

    yield slam
    slam.shutdown()

@pytest.mark.skip(reason="SEGFAULT")
def test_get_current_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.get_current_points()}")

def test_get_frame_pose(slam_in_okay): # PASSING
    logger.debug(f"{slam_in_okay.get_frame_pose()}")

@pytest.mark.skip(reason="SEGFAULT")
def test_get_camera_matrix(slam_in_okay):
    logger.debug(f"{slam_in_okay.get_camera_matrix()}")

@pytest.mark.skip(reason="not SEGFAULT")
def test_get_dist_coef(slam_in_okay):
    logger.debug(f"{slam_in_okay.get_dist_coef()}")

def test_get_keyframe_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.get_keyframe_points()}")

def test_get_keyframe_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.get_keyframe_points()}")
    
@pytest.mark.skip(reason="SEGFAULT")
def test_get_trajectory_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.get_trajectory_points()}")

def test_get_tracked_mappoints(slam_in_okay):
    logger.debug(f"{slam_in_okay.get_tracked_mappoints()}")

def test_get_num_features(slam_in_okay):
    logger.debug(f"{slam_in_okay.get_num_features()}")

@pytest.mark.skip(reason="SEGFAULT")
def test_get_num_matched_features(slam_in_okay):
    logger.debug(f"{slam_in_okay.get_num_matched_features()}")
