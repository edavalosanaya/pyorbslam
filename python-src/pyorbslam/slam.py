import pathlib
import logging
import os
from typing import List, Optional

import yaml
import numpy as np

from . import orbslam3
from .state import State
from .sensor import Sensor

logger = logging.getLogger("pyorbslam")
        
# CONSTANTS
SLAM_MAP = {
    Sensor.MONOCULAR: orbslam3.Sensor.MONOCULAR,
    Sensor.MONOCULAR_IMU: orbslam3.Sensor.IMU_MONOCULAR,
    Sensor.STEREO: orbslam3.Sensor.STEREO,
    Sensor.STEREO_IMU: orbslam3.Sensor.IMU_STEREO,
    Sensor.RGBD: orbslam3.Sensor.RGBD,
}

STATE_MAP = {
    orbslam3.TrackingState.OK: State.OK,
    orbslam3.TrackingState.LOST: State.LOST,
    orbslam3.TrackingState.NOT_INITIALIZED: State.NOT_INITIALIZED,
    orbslam3.TrackingState.SYSTEM_NOT_READY: State.SYSTEM_NOT_READY,
}

CWD = pathlib.Path(os.path.abspath(__file__)).parent
DEFAULT_VOCAB = CWD.parent.parent / 'src' / 'ORB_SLAM3' / 'Vocabulary' / 'ORBvoc.txt'


class ASLAM:

    slam: orbslam3.System

    def __init__(
            self, 
            config_file: pathlib.Path, 
            vocab_file: pathlib.Path = DEFAULT_VOCAB, 
            use_viewer: bool = False
        ):

        # Save the input parameters
        self.vocab_file = vocab_file
        self.config_file = config_file
        self.use_viewer = use_viewer

        # Container
        self.image: np.ndarray = np.empty((0,0,3))
        self.pose_array: List[np.ndarray] = []

        # check the existence of the configuration file
        if not config_file.exists():
            raise FileNotFoundError(f"{config_file} not found")

        if not vocab_file.exists():
            raise FileNotFoundError(f"{vocab_file} not found")

        # Load some configurations
        with open(self.config_file) as f:
            f.readline() # Skip the %YAML:1.0 header as it causes yaml to fail
            config = yaml.safe_load(f)

        self.camera_width = config['Camera.width']
        self.camera_height = config['Camera.height']

    def get_pose_to_target(self, precedent_frame: int = -1) -> Optional[np.ndarray]:
        """Get the pose between a previous frame X in the sequence and the current one T.

        The param `precedent_frame` allows to distinguish between different situations:
        * precedent_frame = -1:  X is the first frame of the sequence. We get the pose between 0->T
        * precedent_frame > 0 :  X is frame at (T-precedent_frame). For instance, if precedent_frame=1, it means we are computing the
        pose  (T-1) -> T.

        Args:
            precedent_frame : id of the frame to use when computing the pose between frame. Default is -1 (i.e., pose 0->T)

        Returns:
            the 4x4 pose matrix corresponding to the transormation between the precedent_frame and the current one.
            If the state is not State.OK, return None


        Examples:
            >>> slam.get_pose_to_target() # return the pose 0 -> T
            >>> slam.get_pose_to_target(precedent_frame=1) # return the pose (T-1) -> T
            >>> slam.get_pose_to_target(precedent_frame=2) # return the pose (T-2) -> T

        """
        if self.slam.get_tracking_state() == orbslam3.TrackingState.OK:
            return np.linalg.inv(self.pose_array[precedent_frame])

        return None

    def get_abs_cloud(self):
        if self.slam.get_tracking_state() == orbslam3.TrackingState.OK:
            return self.slam.get_tracked_mappoints()

    def get_camera_matrix(self):
        return self.slam.get_camera_matrix()

    def get_state(self):
        try:
            return STATE_MAP[self.slam.get_tracking_state()]
        except KeyError:
            return State.LOST

    def get_pose_from_target(self):
        """Get the pose from the current frame T to the reference one 0."""
        if self.get_state() == State.OK:
            return np.linalg.inv(self.pose_array[-1])
        return None
    
    def get_point_cloud(self):
        """Get the point cloud at the current frame form the wiev of the current position .

        Return:
            an array with the 3D coordinate of the point, None if the traking is failed

        """
        if self.get_state() == State.OK:
            return [cp for (cp, _) in self._get_2d_point()]
        return None

    def get_point_cloud_colored(self):
        """Get the point cloud at the current frame form the wiev of the current position with the RGB color of the point .

        Return:
            an array with the 3D coordinate of the point and the relative RGB color, None if the traking is failed

        """
        if self.get_state() == State.OK:
            return [
                [cp, self.image[point[1], point[0]]]
                for (cp, point) in self._get_2d_point()
            ]
        return None

    def get_depth(self):
        """Get the depth computed in the current image.

        Return:
            an array of the image shape with depth when it is aviable otherwise -1 , None if the traking is failed

        """
        depth = None
        if self.get_state() == State.OK:
            depth = np.ones((self.camera_height, self.camera_width)) * -1
            for (cp, point) in self._get_2d_point():
                depth[point[1], point[0]] = cp[2]
        return depth

    def _get_2d_point(self):
        """This private method is used to compute the transormation between the absolute point to the image point

        Return:
            a np.ndarray of pairs (camera view, image point) , an empty list if the tracking is failed

        """
        points2D = []
        points = self.get_abs_cloud()
        camera_matrix = self.slam.get_camera_matrix()
        pose = self.get_pose_from_target()
        for point in points:
            point = np.append(point, [1]).reshape(4, 1)
            camera_points = np.dot(pose, point)
            if camera_points[2] >= 0:
                u = (
                    camera_matrix[0, 0] * (camera_points[0] / camera_points[2])
                    + camera_matrix[0, 2]
                )
                v = (
                    camera_matrix[1, 1] * (camera_points[1] / camera_points[2])
                    + camera_matrix[1, 2]
                )
            if int(v) in range(0, self.camera_height):
                if int(u) in range(0, self.camera_width):
                    points2D.append([camera_points, (int(u), int(v))])
        return points2D

    def reset(self):
        self.slam.reset()
        self.pose_array = []

    def shutdown(self):
        self.slam.shutdown()
        self.pose_array = []

class MonoSLAM(ASLAM):

    def __init__(
            self, 
            config_file: pathlib.Path, 
            vocab_file: pathlib.Path = DEFAULT_VOCAB, 
            use_viewer: bool = False
        ):
        super().__init__(vocab_file=vocab_file, config_file=config_file, use_viewer=use_viewer)
        
        # Init ORB-SLAM3
        self.slam = orbslam3.System(
            str(vocab_file), str(config_file), orbslam3.Sensor.MONOCULAR
        )

        self.slam.set_use_viewer(self.use_viewer)
        self.slam.initialize()

    def process(self, image: np.ndarray, tframe: float):
        """Process an image mono.

        Args:
            image : ndarray of the image
            tframe (float): the timestamp when the image was capture

        Returns:
            the state of the tracking in this frame

        """
        self.image = image
        pose = self.slam.process_image_mono(image, tframe, "0")
        if self.get_state() == State.OK:
            self.pose_array.append(pose)
        return self.get_state()

class StereoSLAM(ASLAM):
    
    def __init__(
            self, 
            config_file: pathlib.Path, 
            vocab_file: pathlib.Path = DEFAULT_VOCAB, 
            use_viewer: bool = False
        ):
        super().__init__(vocab_file=vocab_file, config_file=config_file, use_viewer=use_viewer)
        
        # Init ORB-SLAM3
        self.slam = orbslam3.System(
            vocab_file, config_file, orbslam3.Sensor.STEREO
        )

        self.slam.set_use_viewer(self.use_viewer)
        self.slam.initialize()

    def process(self, image_left: np.ndarray, image_right: np.ndarray, tframe: float):
        """Process a stereo pair.

        Args:
            image_left (ndarray) : left image as HxWx3
            image_right (ndarray) : right image as HxWx3
            tframe (float): the timestamp when the image was capture

        Returns:
            the state of the traking in this frame

        """
        self.image = image_left
        self.slam.process_image_stereo(image_left, image_right, tframe, "0")
        if self.get_state() == State.OK:
            self.pose_array.append(self.get_pose_to_target())
        return self.get_state()

class MonoIMUSLAM(ASLAM):
    
    def __init__(
            self, 
            config_file: pathlib.Path, 
            vocab_file: pathlib.Path = DEFAULT_VOCAB, 
            use_viewer: bool = False
        ):
        super().__init__(vocab_file=vocab_file, config_file=config_file, use_viewer=use_viewer)
        
        # Init ORB-SLAM3
        self.slam = orbslam3.System(
            vocab_file, config_file, orbslam3.Sensor.MONOCULAR_IMU
        )

        self.slam.set_use_viewer(self.use_viewer)
        self.slam.initialize()

    def process(self, image: np.ndarray, tframe: float, imu: np.ndarray):
        """Process an image mono with the imu data.

        Args:
            image (ndarray): image as HxWx3
            tframe (float): the timestamp when the image was capture
            imu : the imu data stored in an float array in the form of [ AccX ,AccY ,AccZ, GyroX, vGyroY, vGyroZ, Timestamp]

        Returns:
            the state of the traking in this frame

        """
        self.image = image
        self.slam.process_image_imu_mono(image, tframe, "0", imu)
        if self.get_state() == State.OK:
            self.pose_array.append(self.get_pose_to_target())
        return self.get_state()

class StereoIMUSLAM(ASLAM):
    
    def __init__(
            self, 
            config_file: pathlib.Path, 
            vocab_file: pathlib.Path = DEFAULT_VOCAB, 
            use_viewer: bool = False
        ):
        super().__init__(vocab_file=vocab_file, config_file=config_file, use_viewer=use_viewer)
        
        # Init ORB-SLAM3
        self.slam = orbslam3.System(
            vocab_file, config_file, orbslam3.Sensor.STEREO_IMU
        )

        self.slam.set_use_viewer(self.use_viewer)
        self.slam.initialize()

    def process(self, image_left: np.ndarray, image_right: np.ndarray, tframe: float, imu: np.ndarray):
        """Process an image stereo with the imu data.

        Args:
            image_left (ndarray): left image as HxWx3
            image_right (ndarray) : right image as HxWx3
            tframe (float): the timestamp when the image was capture
            imu : the imu data stored in an float array in the form of [ AccX ,AccY ,AccZ, GyroX, vGyroY, vGyroZ, Timestamp]

        Returns:
            the state of the traking in this frame

        """
        self.image = image_left
        self.slam.process_image_imu_stereo(
            image_left, image_right, tframe, "0", imu
        )
        if self.get_state() == State.OK:
            self.pose_array.append(self.get_pose_to_target())
        return self.get_state()

class RgbdSLAM(ASLAM):
    
    def __init__(
            self, 
            config_file: pathlib.Path, 
            vocab_file: pathlib.Path = DEFAULT_VOCAB, 
            use_viewer: bool = False
        ):
        super().__init__(vocab_file=vocab_file, config_file=config_file, use_viewer=use_viewer)
        
        # Init ORB-SLAM3
        self.slam = orbslam3.System(
            vocab_file, config_file, orbslam3.Sensor.RGBD
        )

        self.slam.set_use_viewer(self.use_viewer)
        self.slam.initialize()

    def process_image_rgbd(self, image, tframe):
        """Process an  rgbd image.

        Args:
            image (ndarray): RGBD image as HxWx4
            tframe (float): the timestamp when the image was capture

        Returns:
            the new state of the SLAM system

        """
        self.image = image
        self.slam.process_image_rgbd(image, tframe, "0")
        if self.get_state() == State.OK:
            self.pose_array.append(self.get_pose_to_target())
        return self.get_state()
