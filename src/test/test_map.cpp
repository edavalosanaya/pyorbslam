/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<filesystem>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<gtest/gtest.h>

#include<opencv2/core/core.hpp>

#include<ORB_SLAM3/Atlas.h>
#include<ORB_SLAM3/KeyFrame.h>
#include<ORB_SLAM3/KeyFrameDatabase.h>


using namespace std;

TEST(ORBSLAM3UnitTest, TestCreateMapPoint) {
    
    std::filesystem::path currentPath = std::filesystem::absolute(__FILE__).parent_path();
    std::filesystem::path VOCAB_PATH = currentPath / "../ORB_SLAM3/Vocabulary/ORBvoc.txt";

    ORB_SLAM3::Atlas* mpAtlas = new ORB_SLAM3::Atlas();
    std::cout << "Created Atlas" << std::endl;

    ORB_SLAM3::ORBVocabulary* mpVocabulary = new ORB_SLAM3::ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(VOCAB_PATH.string());
    if(!bVocLoad)
    {
        std::cerr << "Wrong path to vocabulary. " << std::endl;
        std::cerr << "Falied to open at: " << VOCAB_PATH.string() << std::endl;
        exit(-1);
    }
    std::cout << "Vocabulary loaded!" << std::endl << std::endl;

    //Create KeyFrame Database
    ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDB = new ORB_SLAM3::KeyFrameDatabase(*mpVocabulary);
    std::cout << "Created KeyFrameDatabase" << std::endl;

    // Create currentFrame
    /* ORB_SLAM3::Frame mCurrentFrame = ORB_SLAM3::Frame(); */
    ORB_SLAM3::Frame mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    std::cout << "Created Frame" << std::endl;

    // Create KeyFrame
    shared_ptr<ORB_SLAM3::KeyFrame> kf = make_shared<ORB_SLAM3::KeyFrame>(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    std::cout << "Create KeyFrame" << std::endl;

    /* // Insert KeyFrame in the map */
    /* mpAtlas->AddKeyFrame(pKFini); */

    /* Eigen::Vector3f x3D; */
    /* mCurrentFrame.UnprojectStereo(i, x3D); */
    /* shared_ptr<MapPoint> pNewMP = make_shared<MapPoint>(x3D, pKFini, mpAtlas->GetCurrentMap()); */

}
