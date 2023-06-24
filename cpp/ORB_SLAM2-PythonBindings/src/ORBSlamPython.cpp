#define PY_ARRAY_UNIQUE_SYMBOL pbcvt_ARRAY_API
#include <opencv2/core/core.hpp>
#include <pyboostcvconverter/pyboostcvconverter.hpp>
#include <ORB_SLAM3/KeyFrame.h>
#include <ORB_SLAM3/Converter.h>
#include <ORB_SLAM3/Tracking.h>
#include <ORB_SLAM3/MapPoint.h>
#include "ORBSlamPython.h"
#if PY_VERSION_HEX >= 0x03000000
#define NUMPY_IMPORT_ARRAY_RETVAL NULL
#else
#define NUMPY_IMPORT_ARRAY_RETVAL
#endif

#if (PY_VERSION_HEX >= 0x03000000)
static void *init_ar()
{
#else
static void init_ar()
{
#endif
    Py_Initialize();
    import_array();
    return NUMPY_IMPORT_ARRAY_RETVAL;
}

BOOST_PYTHON_MODULE(orbslam3)
{
    init_ar();
    boost::python::numpy::initialize();
    boost::python::to_python_converter<cv::Mat, pbcvt::matToNDArrayBoostConverter>();
    pbcvt::matFromNDArrayBoostConverter();

    boost::python::enum_<ORB_SLAM3::Tracking::eTrackingState>("TrackingState")
        .value("SYSTEM_NOT_READY", ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY)
        .value("NO_IMAGES_YET", ORB_SLAM3::Tracking::eTrackingState::NO_IMAGES_YET)
        .value("NOT_INITIALIZED", ORB_SLAM3::Tracking::eTrackingState::NOT_INITIALIZED)
        .value("OK", ORB_SLAM3::Tracking::eTrackingState::OK)
        .value("LOST", ORB_SLAM3::Tracking::eTrackingState::LOST);

    boost::python::enum_<ORB_SLAM3::System::eSensor>("Sensor")
        .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
        .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
        .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO);

    boost::python::class_<ORBSlamPython, boost::noncopyable>("System", boost::python::init<const char *, const char *, boost::python::optional<ORB_SLAM3::System::eSensor>>())
        .def(boost::python::init<std::string, std::string, boost::python::optional<ORB_SLAM3::System::eSensor>>())
        .def("initialize", &ORBSlamPython::initialize)
        .def("load_and_process_mono", &ORBSlamPython::loadAndProcessMono)
        .def("process_image_mono", &ORBSlamPython::processMono)
        .def("load_and_process_imu_mono", &ORBSlamPython::loadAndProcessImuMono)
        .def("process_image_imu_mono", &ORBSlamPython::processImuMono)
        .def("load_and_process_stereo", &ORBSlamPython::loadAndProcessStereo)
        .def("process_image_stereo", &ORBSlamPython::processStereo)
        .def("load_and_process_imu_stereo", &ORBSlamPython::loadAndProcessImuStereo)
        .def("process_image_imu_stereo", &ORBSlamPython::processImuStereo)
        .def("load_and_process_rgbd", &ORBSlamPython::loadAndProcessRGBD)
        .def("process_image_rgbd", &ORBSlamPython::processRGBD)
        .def("shutdown", &ORBSlamPython::shutdown)
        .def("is_running", &ORBSlamPython::isRunning)
        .def("reset", &ORBSlamPython::reset)
        .def("activateSLAM", &ORBSlamPython::activateSLAMTraking)
        .def("deactivateSLAM", &ORBSlamPython::deactivateSLAMTraking)
        .def("get_current_points", &ORBSlamPython::getCurrentPoints)
        .def("get_frame_pose", &ORBSlamPython::getFramePose)
        .def("get_camera_matrix", &ORBSlamPython::getCameraMatrix)
        .def("get_dist_coef", &ORBSlamPython::getDistCoeff)
        .def("set_mode", &ORBSlamPython::setMode)
        .def("set_use_viewer", &ORBSlamPython::setUseViewer)
        .def("get_keyframe_points", &ORBSlamPython::getKeyframePoints)
        .def("get_trajectory_points", &ORBSlamPython::getTrajectoryPoints)
        .def("get_tracked_mappoints", &ORBSlamPython::getTrackedMappoints)
        .def("get_tracking_state", &ORBSlamPython::getTrackingState)
        .def("get_num_features", &ORBSlamPython::getNumFeatures)
        .def("get_num_matched_features", &ORBSlamPython::getNumMatches)
        .def("save_settings", &ORBSlamPython::saveSettings)
        .def("load_settings", &ORBSlamPython::loadSettings)
        .def("save_settings_file", &ORBSlamPython::saveSettingsFile)
        .staticmethod("save_settings_file")
        .def("load_settings_file", &ORBSlamPython::loadSettingsFile)
        .staticmethod("load_settings_file");
}

ORBSlamPython::ORBSlamPython(std::string vocabFile, std::string settingsFile, ORB_SLAM3::System::eSensor sensorMode)
    : vocabluaryFile(vocabFile),
      settingsFile(settingsFile),
      sensorMode(sensorMode),
      system(nullptr),
      bUseViewer(false),
      bUseRGB(true)
{
}

ORBSlamPython::ORBSlamPython(const char *vocabFile, const char *settingsFile, ORB_SLAM3::System::eSensor sensorMode)
    : vocabluaryFile(vocabFile),
      settingsFile(settingsFile),
      sensorMode(sensorMode),
      system(nullptr),
      bUseViewer(false),
      bUseRGB(true)
{
}

ORBSlamPython::~ORBSlamPython()
{
}

bool ORBSlamPython::initialize()
{
    system = std::make_shared<ORB_SLAM3::System>(vocabluaryFile, settingsFile, sensorMode, bUseViewer);
    return true;
}

bool ORBSlamPython::isRunning()
{
    return system != nullptr;
}

void ORBSlamPython::reset()
{
    if (system)
    {
        system->Reset();
    }
}

bool ORBSlamPython::loadAndProcessMono(std::string imageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    return this->processMono(im, timestamp, imageFile);
}
// helper function to convert ndarray to vector<ORB_SLAM3::IMU::Point>

vector<ORB_SLAM3::IMU::Point> convertImuFromNDArray(boost::python::numpy::ndarray imu);

bool ORBSlamPython::processMono(cv::Mat image, double timestamp, std::string imageFile)
{
    if (!system)
    {
        return false;
    }
    if (image.data)
    {
        cv::Mat pose = system->TrackMonocular(image, timestamp, vector<ORB_SLAM3::IMU::Point>(), imageFile);
        return !pose.empty();
    }
    else
    {
        return false;
    }
}

bool ORBSlamPython::loadAndProcessImuMono(std::string imageFile, double timestamp, boost::python::numpy::ndarray imu)
{

    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    return this->processImuMono(im, timestamp, imageFile, imu);
}

bool ORBSlamPython::processImuMono(cv::Mat image, double timestamp, std::string imageFile, boost::python::numpy::ndarray imu)
{
    if (!system)
    {
        return false;
    }
    if (image.data)
    {
        vector<ORB_SLAM3::IMU::Point> vImuMeas = convertImuFromNDArray(imu);
        cv::Mat pose = system->TrackMonocular(image, timestamp, vImuMeas);
        return !pose.empty();
    }
    else
    {
        return false;
    }
}

bool ORBSlamPython::loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat leftImage = cv::imread(leftImageFile, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(rightImageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2RGB);
        cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2RGB);
    }
    return this->processStereo(leftImage, rightImage, timestamp);
}

bool ORBSlamPython::processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp)
{
    if (!system)
    {
        return false;
    }
    if (leftImage.data && rightImage.data)
    {
        cv::Mat pose = system->TrackStereo(leftImage, rightImage, timestamp);
        return !pose.empty();
    }
    else
    {
        return false;
    }
}

bool ORBSlamPython::loadAndProcessImuStereo(std::string leftImageFile, std::string rightImageFile, double timestamp, boost::python::numpy::ndarray imu)
{
    if (!system)
    {
        return false;
    }
    cv::Mat leftImage = cv::imread(leftImageFile, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(rightImageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2RGB);
        cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2RGB);
    }
    return this->processImuStereo(leftImage, rightImage, timestamp, imu);
}

bool ORBSlamPython::processImuStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp, boost::python::numpy::ndarray imu)
{
    if (!system)
    {
        return false;
    }
    if (leftImage.data && rightImage.data)
    {
        vector<ORB_SLAM3::IMU::Point> vImuMeas = convertImuFromNDArray(imu);
        cv::Mat pose = system->TrackStereo(leftImage, rightImage, timestamp, vImuMeas);
        return !pose.empty();
    }
    else
    {
        return false;
    }
}

bool ORBSlamPython::loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    cv::Mat imDepth = cv::imread(depthImageFile, cv::IMREAD_UNCHANGED);
    return this->processRGBD(im, imDepth, timestamp);
}

bool ORBSlamPython::processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp)
{
    if (!system)
    {
        return false;
    }
    if (image.data && depthImage.data)
    {
        cv::Mat pose = system->TrackRGBD(image, depthImage, timestamp);
        return !pose.empty();
    }
    else
    {
        return false;
    }
}

void ORBSlamPython::shutdown()
{
    if (system)
    {
        system->Shutdown();
        system.reset();
    }
}

void ORBSlamPython::activateSLAMTraking()
{
    if (system)
    {
        system->ActivateLocalizationMode();
    }
}

void ORBSlamPython::deactivateSLAMTraking()
{
    if (system)
    {
        system->DeactivateLocalizationMode();
    }
}

ORB_SLAM3::Tracking::eTrackingState ORBSlamPython::getTrackingState() const
{
    if (system)
    {
        return static_cast<ORB_SLAM3::Tracking::eTrackingState>(system->GetTrackingState());
    }
    return ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY;
}

unsigned int ORBSlamPython::getNumFeatures() const
{
    if (system)
    {
        return system->GetTracker()->mCurrentFrame.mvKeys.size();
    }
    return 0;
}

unsigned int ORBSlamPython::getNumMatches() const
{
    if (system)
    {
        // This code is based on the display code in FrameDrawer.cc, with a little extra safety logic to check the length of the vectors.
        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        unsigned int matches = 0;
        unsigned int num = pTracker->mCurrentFrame.mvKeys.size();
        if (pTracker->mCurrentFrame.mvpMapPoints.size() < num)
        {
            num = pTracker->mCurrentFrame.mvpMapPoints.size();
        }
        if (pTracker->mCurrentFrame.mvbOutlier.size() < num)
        {
            num = pTracker->mCurrentFrame.mvbOutlier.size();
        }
        for (unsigned int i = 0; i < num; ++i)
        {
            ORB_SLAM3::MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if (pMP && !pTracker->mCurrentFrame.mvbOutlier[i] && pMP->Observations() > 0)
            {
                ++matches;
            }
        }
        return matches;
    }
    return 0;
}

boost::python::list ORBSlamPython::getKeyframePoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    // This is copied from the ORB_SLAM3 System.SaveKeyFrameTrajectoryTUM function, with some changes to output a python tuple.
    vector<ORB_SLAM3::KeyFrame *> vpKFs = system->GetKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    boost::python::list trajectory;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        ORB_SLAM3::KeyFrame *pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if (pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        cv::Mat t = pKF->GetCameraCenter();
        PyObject *Rarr = pbcvt::fromMatToNDArray(R);
        PyObject *Tarr = pbcvt::fromMatToNDArray(t);
        trajectory.append(boost::python::make_tuple(
            pKF->mTimeStamp,
            boost::python::handle<>(Rarr),
            boost::python::handle<>(Tarr)));
    }

    return trajectory;
}

boost::python::list ORBSlamPython::getTrackedMappoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    // This is copied from the ORB_SLAM3 System.SaveTrajectoryKITTI function, with some changes to output a python tuple.
    vector<ORB_SLAM3::MapPoint *> Mps = system->GetTrackedMapPoints();

    boost::python::list map_points;
    for (size_t i = 0; i < Mps.size(); i++)
    {
        if (Mps[i] != NULL)
        {
            cv::Mat wp = Mps[i]->GetWorldPos();
            map_points.append(boost::python::make_tuple(
                wp.at<float>(0, 0),
                wp.at<float>(1, 0),
                wp.at<float>(2, 0)));
        }
    }

    return map_points;
}

boost::python::list ORBSlamPython::getCurrentPoints() const
{
    if (system)
    {

        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        boost::python::list map_points;
        unsigned int num = pTracker->mCurrentFrame.mvKeys.size();
        vector<cv::KeyPoint> Kps = pTracker->mCurrentFrame.mvKeysUn;
        if (pTracker->mCurrentFrame.mvpMapPoints.size() < num)
        {
            num = pTracker->mCurrentFrame.mvpMapPoints.size();
        }
        if (pTracker->mCurrentFrame.mvbOutlier.size() < num)
        {
            num = pTracker->mCurrentFrame.mvbOutlier.size();
        }
        for (unsigned int i = 0; i < num; ++i)
        {
            ORB_SLAM3::MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if (pMP && !pTracker->mCurrentFrame.mvbOutlier[i] && pMP->Observations() > 0)
            {
                cv::Mat wp = pMP->GetWorldPos();
                map_points.append(boost::python::make_tuple(
                    boost::python::make_tuple(
                        wp.at<float>(0, 0),
                        wp.at<float>(1, 0),
                        wp.at<float>(2, 0)),
                    boost::python::make_tuple(
                        Kps[i].pt.x,
                        Kps[i].pt.y)));
            }
        }
        return map_points;
    }
    return boost::python::list();
}

PyObject *ORBSlamPython::getFramePose() const
{
    if (system)
    {

        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        cv::Mat pose = pTracker->mCurrentFrame.mTcw;
        if (pose.rows * pose.cols > 0)
        {
            return pbcvt::fromMatToNDArray(pose);
        }
    }
    return NULL;
}

PyObject *ORBSlamPython::getCameraMatrix() const
{
    if (system)
    {

        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        cv::Mat cm = pTracker->mCurrentFrame.mK;
        return pbcvt::fromMatToNDArray(cm);
    }
    return NULL;
}

boost::python::tuple ORBSlamPython::getDistCoeff() const
{
    if (system)
    {

        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        cv::Mat dist = pTracker->mCurrentFrame.mDistCoef;
        return boost::python::make_tuple(
            dist.at<float>(0),
            dist.at<float>(1),
            dist.at<float>(2),
            dist.at<float>(3));
    }
    return boost::python::make_tuple();
}

boost::python::list ORBSlamPython::getTrajectoryPoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    // This is copied from the ORB_SLAM3 System.SaveTrajectoryKITTI function, with some changes to output a python tuple.
    vector<ORB_SLAM3::KeyFrame *> vpKFs = system->GetKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    // Of course, if we have no keyframes, then just use the identity matrix.
    cv::Mat Two = cv::Mat::eye(4, 4, CV_32F);
    if (vpKFs.size() > 0)
    {
        cv::Mat Two = vpKFs[0]->GetPoseInverse();
    }

    boost::python::list trajectory;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFrame *>::iterator lRit = system->GetTracker()->mlpReferences.begin();
    std::list<double>::iterator lT = system->GetTracker()->mlFrameTimes.begin();
    for (std::list<cv::Mat>::iterator lit = system->GetTracker()->mlRelativeFramePoses.begin(), lend = system->GetTracker()->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame *pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        while (pKF != NULL && pKF->isBad())
        {
            ORB_SLAM3::KeyFrame *pKFParent;

            // std::cout << "bad parent" << std::endl;
            Trw = Trw * pKF->mTcp;
            pKFParent = pKF->GetParent();
            if (pKFParent == pKF)
            {
                // We've found a frame that is it's own parent, presumably a root or something. Break out
                break;
            }
            else
            {
                pKF = pKFParent;
            }
        }
        if (pKF != NULL && !pKF->isBad())
        {
            Trw = Trw * pKF->GetPose() * Two;

            cv::Mat Tcw = (*lit) * Trw;
            //cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            //cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
            PyObject *ndarr = pbcvt::fromMatToNDArray(Tcw);
            trajectory.append(boost::python::make_tuple(
                *lT,
                boost::python::handle<>(ndarr)));
        }
    }

    return trajectory;
}

void ORBSlamPython::setMode(ORB_SLAM3::System::eSensor mode)
{
    sensorMode = mode;
}

void ORBSlamPython::setUseViewer(bool useViewer)
{
    bUseViewer = useViewer;
}

void ORBSlamPython::setRGBMode(bool rgb)
{
    bUseRGB = rgb;
}

bool ORBSlamPython::saveSettings(boost::python::dict settings) const
{
    return ORBSlamPython::saveSettingsFile(settings, settingsFile);
}

boost::python::dict ORBSlamPython::loadSettings() const
{
    return ORBSlamPython::loadSettingsFile(settingsFile);
}

bool ORBSlamPython::saveSettingsFile(boost::python::dict settings, std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::WRITE);

    boost::python::list keys = settings.keys();
    for (int index = 0; index < boost::python::len(keys); ++index)
    {
        boost::python::extract<std::string> extractedKey(keys[index]);
        if (!extractedKey.check())
        {
            continue;
        }
        std::string key = extractedKey;

        boost::python::extract<int> intValue(settings[key]);
        if (intValue.check())
        {
            fs << key << int(intValue);
            continue;
        }

        boost::python::extract<float> floatValue(settings[key]);
        if (floatValue.check())
        {
            fs << key << float(floatValue);
            continue;
        }

        boost::python::extract<std::string> stringValue(settings[key]);
        if (stringValue.check())
        {
            fs << key << std::string(stringValue);
            continue;
        }
    }

    return true;
}

// Helpers for reading cv::FileNode objects into python objects.
boost::python::list readSequence(cv::FileNode fn, int depth = 10);
boost::python::dict readMap(cv::FileNode fn, int depth = 10);

boost::python::dict ORBSlamPython::loadSettingsFile(std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::READ);
    cv::FileNode root = fs.root();
    if (root.isMap())
    {
        return readMap(root);
    }
    else if (root.isSeq())
    {
        boost::python::dict settings;
        settings["root"] = readSequence(root);
        return settings;
    }
    return boost::python::dict();
}

// ----------- HELPER DEFINITIONS -----------
boost::python::dict readMap(cv::FileNode fn, int depth)
{
    boost::python::dict map;
    if (fn.isMap())
    {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it)
        {
            cv::FileNode item = *it;
            std::string key = item.name();

            if (item.isNone())
            {
                map[key] = boost::python::object();
            }
            else if (item.isInt())
            {
                map[key] = int(item);
            }
            else if (item.isString())
            {
                map[key] = std::string(item);
            }
            else if (item.isReal())
            {
                map[key] = float(item);
            }
            else if (item.isSeq() && depth > 0)
            {
                map[key] = readSequence(item, depth - 1);
            }
            else if (item.isMap() && depth > 0)
            {
                map[key] = readMap(item, depth - 1); // Depth-limited recursive call to read inner maps
            }
        }
    }
    return map;
}

boost::python::list readSequence(cv::FileNode fn, int depth)
{
    boost::python::list sequence;
    if (fn.isSeq())
    {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it)
        {
            cv::FileNode item = *it;

            if (item.isNone())
            {
                sequence.append(boost::python::object());
            }
            else if (item.isInt())
            {
                sequence.append(int(item));
            }
            else if (item.isString())
            {
                sequence.append(std::string(item));
            }
            else if (item.isReal())
            {
                sequence.append(float(item));
            }
            else if (item.isSeq() && depth > 0)
            {
                sequence.append(readSequence(item, depth - 1)); // Depth-limited recursive call to read nested sequences
            }
            else if (item.isMap() && depth > 0)
            {
                sequence.append(readMap(item, depth - 1));
            }
        }
    }
    return sequence;
}

vector<ORB_SLAM3::IMU::Point> convertImuFromNDArray(boost::python::numpy::ndarray imu)
{
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    double vAccX, vAccY, vAccZ, vGyroX, vGyroY, vGyroZ;
    float vTimestamp;
    Py_intptr_t const *strides = imu.get_strides();
    for (int i = 0; i < imu.shape(0); i++)
    {
        vAccX = *reinterpret_cast<float const *>(imu.get_data() + i * strides[0] + 0 * strides[1]);
        vAccY = *reinterpret_cast<float const *>(imu.get_data() + i * strides[0] + 1 * strides[1]);
        vAccZ = *reinterpret_cast<float const *>(imu.get_data() + i * strides[0] + 2 * strides[1]);
        vGyroX = *reinterpret_cast<float const *>(imu.get_data() + i * strides[0] + 3 * strides[1]);
        vGyroY = *reinterpret_cast<float const *>(imu.get_data() + i * strides[0] + 4 * strides[1]);
        vGyroZ = *reinterpret_cast<float const *>(imu.get_data() + i * strides[0] + 5 * strides[1]);
        vTimestamp = *reinterpret_cast<double const *>(imu.get_data() + i * strides[0] + 6 * strides[1]);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAccX, vAccY, vAccZ, vGyroX, vGyroY, vGyroZ, vTimestamp));
    }
    return vImuMeas;
}
