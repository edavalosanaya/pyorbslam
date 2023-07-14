import logging

logger = logging.getLogger("pyorbslam")

class TestEntryPoints():

    def test_get_current_points(self, slam_in_okay):
        logger.debug(f"{slam_in_okay.slam.get_current_points()}")

    def test_get_frame_pose(self, slam_in_okay): # PASSING
        logger.debug(f"{slam_in_okay.slam.get_frame_pose()}")

    def test_get_camera_matrix(self, slam_in_okay):
        logger.debug(f"{slam_in_okay.slam.get_camera_matrix()}")

    def test_get_dist_coef(self, slam_in_okay):
        logger.debug(f"{slam_in_okay.slam.get_dist_coef()}")

    def test_get_keyframe_points(self, slam_in_okay):
        logger.debug(f"{slam_in_okay.slam.get_keyframe_points()}")
        
    def test_get_trajectory_points(self, slam_in_okay):
        logger.debug(f"{slam_in_okay.slam.get_trajectory_points()}")

    def test_get_tracked_mappoints(self, slam_in_okay):
        logger.debug(f"{slam_in_okay.slam.get_tracked_mappoints()}")

    def test_get_num_features(self, slam_in_okay):
        logger.debug(f"{slam_in_okay.slam.get_num_features()}")

    def test_get_num_matched_features(self, slam_in_okay):
        logger.debug(f"{slam_in_okay.slam.get_num_matched_features()}")
