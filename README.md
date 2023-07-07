# pyorbslam

Python-bindings to the real-time ORBSLAM3 C++ library, with the goal of creating a PyPI package soon!

> :warning: This repo is still in active development, without a proper release. You have been warned!

## Installation

As of now, the only OS tested is Ubuntu 22.04. For Windows or MacOS, it is possible to install the library but your mileage may vary.

First, you need to install the dependencies with the following command:

```bash
sudo apt-get install libopencv-dev libeigen3-dev
```

Then clone the repo and install the package. Make sure that you have CMake and gcc update to date.

```bash
git clone https://github.com/edavalosanaya/pyorbslam.git
pip install [-e] .
```

## Usage

```python
# Import the necessary packages
import cv2
import pyorbslam

# Load video and create 3D path drawer
cap = cv2.VideoCapture("video/path/here", 0)
drawer = pyorbslam.TrajectoryDrawer()

# Timestamp information
timestamp = 0
fps = 1/24

# Create SLAM
slam = pyorbslam.MonoSLAM("settings/path/here") # Examples found in ``settings`` folder

# Main loop
while True

    # Read
    ret, frame = cap.read()
    if not ret:
        break

    # Process frame and update timestamp
    state = tobii_slam.process(frame, timestamp)
    timestamp += fps

    # If all things work, then we get a camera pose (YAY!)
    if state == pyorbslam.State.OK:
        pose = tobii_slam.get_pose_to_target()
        drawer.plot_trajectory(pose)

    # Show the image
    cv2.imshow('frame', frame)
    cv2.waitKey(1)
```

## Acknowledgements

This repo uses the original [ORB-SLAM3]((https://github.com/UZ-SLAMLab/ORB_SLAM3)) code, along borrowing much of the code by [GiordanoLaminetti](https://github.com/GiordanoLaminetti) from his modified [ORB-SLAM3](https://github.com/GiordanoLaminetti/ORB_SLAM3) code, [Python bindings](https://github.com/GiordanoLaminetti/ORB_SLAM2-PythonBindings) and [SlamPy python package](https://github.com/GiordanoLaminetti/SlamPy). Many thanks for their work!

## License

This project, along with many of its dependencies, is released under GPLv3 license.


