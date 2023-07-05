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


using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

TEST(ORBSLAM3Test, TestMonocularSLAM) {

    std::filesystem::path currentPath = std::filesystem::absolute(__FILE__).parent_path();
    std::filesystem::path relativePath1 = "./EuRoC.yaml";
    std::filesystem::path VOCAB_PATH = std::filesystem::absolute(currentPath / "../ORB_SLAM3/Vocabulary/ORBvoc.txt");
    std::filesystem::path SETTINGS_FILE = std::filesystem::absolute(currentPath / relativePath1);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(VOCAB_PATH.string(),SETTINGS_FILE.string(),ORB_SLAM3::System::MONOCULAR, false);
    SLAM.Shutdown();
}

// int main(int argc, char **argv)
// {  
//     if(argc < 5)
//     {
//         cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
//         return 1;
//     }

//     const int num_seq = (argc-3)/2;
//     cout << "num_seq = " << num_seq << endl;
//     bool bFileName= (((argc-3) % 2) == 1);
//     string file_name;
//     if (bFileName)
//     {
//         file_name = string(argv[argc-1]);
//         cout << "file name: " << file_name << endl;
//     }

//     // Load all sequences:
//     int seq;
//     vector< vector<string> > vstrImageFilenames;
//     vector< vector<double> > vTimestampsCam;
//     vector<int> nImages;

//     vstrImageFilenames.resize(num_seq);
//     vTimestampsCam.resize(num_seq);
//     nImages.resize(num_seq);

//     int tot_images = 0;
//     for (seq = 0; seq<num_seq; seq++)
//     {
//         cout << "Loading images for sequence " << seq << "...";
//         LoadImages(string(argv[(2*seq)+3]) + "/mav0/cam0/data", string(argv[(2*seq)+4]), vstrImageFilenames[seq], vTimestampsCam[seq]);
//         cout << "LOADED!" << endl;

//         nImages[seq] = vstrImageFilenames[seq].size();
//         tot_images += nImages[seq];
//     }

//     // Vector for tracking time statistics
//     vector<float> vTimesTrack;
//     vTimesTrack.resize(tot_images);

//     cout << endl << "-------" << endl;
//     cout.precision(17);


//     int fps = 20;
//     float dT = 1.f/fps;
//     // Create SLAM system. It initializes all system threads and gets ready to process frames.
//     ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, false);
//     float imageScale = SLAM.GetImageScale();

//     double t_resize = 0.f;
//     double t_track = 0.f;

//     for (seq = 0; seq<num_seq; seq++)
//     {

//         // Main loop
//         cv::Mat im;
//         int proccIm = 0;
//         for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
//         {

//             // Read image from file
//             im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //,CV_LOAD_IMAGE_UNCHANGED);
//             double tframe = vTimestampsCam[seq][ni];

//             if(im.empty())
//             {
//                 cerr << endl << "Failed to load image at: "
//                      <<  vstrImageFilenames[seq][ni] << endl;
//                 return 1;
//             }

//             if(imageScale != 1.f)
//             {
//                 int width = im.cols * imageScale;
//                 int height = im.rows * imageScale;
//                 cv::resize(im, im, cv::Size(width, height));
//                 t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
//                 SLAM.InsertResizeTime(t_resize);

//             // Pass the image to the SLAM system
//             // cout << "tframe = " << tframe << endl;
//             SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial

//             t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
//             SLAM.InsertTrackTime(t_track);
//             double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

//             vTimesTrack[ni]=ttrack;

//             // Wait to load the next frame
//             double T=0;
//             if(ni<nImages[seq]-1)
//                 T = vTimestampsCam[seq][ni+1]-tframe;
//             else if(ni>0)
//                 T = tframe-vTimestampsCam[seq][ni-1];

//             //std::cout << "T: " << T << std::endl;
//             //std::cout << "ttrack: " << ttrack << std::endl;

//             if(ttrack<T) {
//                 //std::cout << "usleep: " << (dT-ttrack) << std::endl;
//                 usleep((T-ttrack)*1e6); // 1e6
//             }
//         }

//         if(seq < num_seq - 1)
//         {
//             string kf_file_submap =  "./SubMaps/kf_SubMap_" + std::to_string(seq) + ".txt";
//             string f_file_submap =  "./SubMaps/f_SubMap_" + std::to_string(seq) + ".txt";
//             SLAM.SaveTrajectoryEuRoC(f_file_submap);
//             SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);

//             cout << "Changing the dataset" << endl;

//             SLAM.ChangeDataset();
//         }

//     }
//     // Stop all threads
//     SLAM.Shutdown();

//     return 0;
// }

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
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
    }
}