#include<iostream>
#include<filesystem>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<gtest/gtest.h>

#include<opencv2/core/core.hpp>

#include<ORB_SLAM3/Atlas.h>

using namespace std;

class AtlasTest : public ::testing::Test {
    protected:
        std::shared_ptr<ORB_SLAM3::Atlas> atlas;

        void SetUp() override {
            atlas = std::make_shared<ORB_SLAM3::Atlas>();
        }

        void TearDown() override {
            atlas.reset();
        }
};


// Not leaking memory
TEST_F(AtlasTest, AddMapPoint) {

    // Creating map
    ORB_SLAM3::Map map = ORB_SLAM3::Map();
    atlas->ChangeMap(&map);

    // Create a point
    std::shared_ptr<ORB_SLAM3::MapPoint> point = std::make_shared<ORB_SLAM3::MapPoint>();
    point->UpdateMap(&map);

    // Add the point to the atlas
    atlas->AddMapPoint(point);

    map.clear();
}