#include<iostream>
#include<filesystem>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<gtest/gtest.h>

#include<opencv2/core/core.hpp>

#include<ORB_SLAM3/Map.h>

using namespace std;


// Not leaking memory
TEST(ORBSLAM3UnitTest, TestCreateMapPoint) {

    // Creating map
    std::shared_ptr<ORB_SLAM3::Map> map = std::make_shared<ORB_SLAM3::Map>();

    // Assuming MapPoint is another class in ORB_SLAM3 that you can initialize
    // Here, I'm using a hypothetical constructor for simplicity.
    std::shared_ptr<ORB_SLAM3::MapPoint> point = std::make_shared<ORB_SLAM3::MapPoint>();

    map->AddMapPoint(point);

    // Check that the map now contains the point
    std::vector<std::shared_ptr<ORB_SLAM3::MapPoint>> points = map->GetAllMapPoints();
    EXPECT_EQ(points.size(), 1);
    EXPECT_EQ(points[0], point);

    map.reset();
}