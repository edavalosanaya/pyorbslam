# pyorbslam

# Building from Source

The building process has only been tested in Ubuntu. These are the initial dependencies:

```bash
sudo apt-get install libeigen3-dev libopencv-dev
```

First, build ORB_SLAM3
```bash
cd cpp/ORB_SLAM3
./build.sh
cd build
make install
```

Then, build and install Sophus thirdparty
```bash
cd cpp/ORB_SLAM3/Thirdparty/Sophus
mkdir build
cd build
cmake ..
make -j4
make install
```

Then, build and install the Python bindings
```bash
cd cpp/bindings
mkdir build
cd build
cmake ..
make -j4
make install
```

