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

#include<ORB_SLAM3/System.h>

/* #define DEBUG_NEW new(__FILE__, __LINE__) */
/* #define new DEBUG_NEW */

/*
Tracking.h

// Tracking states
enum eTrackingState{
    SYSTEM_NOT_READY=-1,
    NO_IMAGES_YET=0,
    NOT_INITIALIZED=1,
    OK=2,
    RECENTLY_LOST=3,
    LOST=4,
    OK_KLT=5
};

*/


using namespace std;

void LoadImages(int length, const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

TEST(ORBSLAM3Test, TestMonocularSLAM) {

    std::filesystem::path currentPath = std::filesystem::absolute(__FILE__).parent_path();
    std::filesystem::path VOCAB_PATH = currentPath / "../ORB_SLAM3/Vocabulary/ORBvoc.txt";
    std::filesystem::path SETTINGS_FILE = currentPath / "./EuRoC.yaml";
    std::filesystem::path IMAGE_DATA_DIR = currentPath / "../../test/data/EuRoC/MH01/mav0/cam0/data";
    std::filesystem::path TIMESTAMP_FILE = currentPath / "../../test/data/EuRoC/timestamps.txt";

    std::cout << "Current Path: " << currentPath << std::endl;
    std::cout << "VOCAB_PATH: " << VOCAB_PATH << std::endl;
    std::cout << "SETTINGS_FILE: " << SETTINGS_FILE << std::endl;
    std::cout << "IMAGE_DATA_DIR: " << IMAGE_DATA_DIR << std::endl;
    std::cout << "TIMESTAMP_FILE: " << TIMESTAMP_FILE << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(VOCAB_PATH.string(), SETTINGS_FILE.string(), ORB_SLAM3::System::MONOCULAR, false);
    float imageScale = SLAM.GetImageScale();
    cout << "SLAM LOADED!" << endl;

    // Select the number of images
    int length = 30;

    // Load the images
    vector<string> strImageFilenames;
    vector<double> timestampsCam;
    LoadImages(length, IMAGE_DATA_DIR.string(), TIMESTAMP_FILE.string(), strImageFilenames, timestampsCam);
    cout << "EuRoC Dataset LOADED!" << endl;

    // Main loop
    cv::Mat im;
    int proccIm = 0;
    /* for(int ni=0; ni<strImageFilenames.size(); ni++, proccIm++){ */
    /* int length = strImageFilenames.size(); */
    cout << "Starting!" << endl;
    for(int ni=0; ni<length; ni++, proccIm++){
        cout << "Processing " << ni << "/" << length << endl;

        // Read image from file
        im = cv::imread(strImageFilenames[ni], cv::IMREAD_UNCHANGED); //,CV_LOAD_IMAGE_UNCHANGED);
        double tframe = timestampsCam[ni];

        // If image fail, stop the test
        if(im.empty()){
            FAIL() << "Failed to load image at: "<<  strImageFilenames[ni];
        }

        // Resize if necessary
        if(imageScale != 1.f){
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            }

        // Pass the image to the SLAM system
        cout << "tframe = " << tframe << endl;
        Sophus::SE3f Tcw = SLAM.TrackMonocular(im, tframe); //
        cout << "state: " << SLAM.GetTrackingState() << endl;
        cout << "pose: " << Tcw.matrix() << endl;

        // Trying accessing the data from the tracker
        /* ORB_SLAM3::Tracking *pTracker = SLAM.GetTracker(); */
        /* Sophus::SE3f sophusPose = pTracker->mCurrentFrame.GetPose(); */
        /* cout << "pTracker mCurrentFrame: Sophus::SE3f=" << sophusPose.matrix() << endl; */

        }

    // Shutdown
    SLAM.Shutdown();
}

void LoadImages(int length, const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    int counter = 0;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);

        }

        cout << "Loading image " << std::to_string(counter) << "/" << std::to_string(length) << endl;
        if (counter >= length) break;
        counter += 1;
    }
}
