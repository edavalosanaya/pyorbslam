#include<iostream>
#include<filesystem>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<gtest/gtest.h>

#include<opencv2/core/core.hpp>

#include<ORB_SLAM3/KeyFrame.h>

using namespace std;


// Not leaking memory
TEST(ORBSLAM3UnitTest, TestCreateKeyFrame) {

    // Creating keyframe
    std::shared_ptr<ORB_SLAM3::KeyFrame> kf = std::make_shared<ORB_SLAM3::KeyFrame>();

    // Assuming MapPoint is another class in ORB_SLAM3 that you can initialize
    // Here, I'm using a hypothetical constructor for simplicity.
    std::shared_ptr<ORB_SLAM3::MapPoint> point = std::make_shared<ORB_SLAM3::MapPoint>();

    kf->AddMapPoint(point, 0);
}