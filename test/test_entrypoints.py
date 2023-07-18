import logging

logger = logging.getLogger("pyorbslam")

def test_get_current_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_current_points()}")

def test_get_camera_matrix(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_camera_matrix()}")

def test_get_dist_coef(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_dist_coef()}")

def test_get_keyframe_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_keyframe_points()}")
    
def test_get_trajectory_points(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_trajectory_points()}")

def test_get_tracked_mappoints(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_tracked_mappoints()}")

def test_get_num_features(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_num_features()}")

def test_get_num_matched_features(slam_in_okay):
    logger.debug(f"{slam_in_okay.slam.get_num_matched_features()}")
