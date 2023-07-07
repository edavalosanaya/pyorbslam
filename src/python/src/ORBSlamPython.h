#ifndef ORBSLAMPYTHON_H
#define ORBSLAMPYTHON_H

#include <memory>
#include <Python.h>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <ORB_SLAM3/System.h>
#include <ORB_SLAM3/Tracking.h>

class ORBSlamPython
{
public:
    ORBSlamPython(std::string vocabFile, std::string settingsFile,
                  ORB_SLAM3::System::eSensor sensorMode = ORB_SLAM3::System::eSensor::RGBD);
    ORBSlamPython(const char *vocabFile, const char *settingsFile,
                  ORB_SLAM3::System::eSensor sensorMode = ORB_SLAM3::System::eSensor::RGBD);
    ~ORBSlamPython();

    bool initialize();
    bool isRunning();
    PyObject *loadAndProcessMono(std::string imageFile, double timestamp);
    PyObject *processMono(cv::Mat image, double timestamp, std::string imageFile);
    PyObject *loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp);
    PyObject *processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp);
    PyObject *loadAndProcessImuMono(std::string imageFile, double timestamp, boost::python::numpy::ndarray imu);
    PyObject *processImuMono(cv::Mat image, double timestamp, std::string imageFile, boost::python::numpy::ndarray imu);
    PyObject *loadAndProcessImuStereo(std::string leftImageFile, std::string rightImageFile, double timestamp, boost::python::numpy::ndarray imu);
    PyObject *processImuStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp, boost::python::numpy::ndarray imu);
    PyObject *loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, double timestamp);
    PyObject *processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp);
    void reset();
    void shutdown();
    void activateSLAMTraking();
    void deactivateSLAMTraking();
    boost::python::list getCurrentPoints() const;
    ORB_SLAM3::Tracking::eTrackingState getTrackingState() const;
    PyObject *getCameraMatrix() const;
    unsigned int getNumFeatures() const;
    unsigned int getNumMatches() const;
    boost::python::tuple getDistCoeff() const;
    boost::python::list getKeyframePoints() const;
    boost::python::list getTrajectoryPoints() const;
    boost::python::list getTrackedMappoints() const;
    bool saveSettings(boost::python::dict settings) const;
    boost::python::dict loadSettings() const;
    void setMode(ORB_SLAM3::System::eSensor mode);
    void setRGBMode(bool rgb);
    void setUseViewer(bool useViewer);

    static bool saveSettingsFile(boost::python::dict settings, std::string settingsFilename);
    static boost::python::dict loadSettingsFile(std::string settingsFilename);

private:
    std::string vocabluaryFile;
    std::string settingsFile;
    ORB_SLAM3::System::eSensor sensorMode;
    std::shared_ptr<ORB_SLAM3::System> system;
    bool bUseViewer;
    bool bUseRGB;
};

#endif // ORBSLAMPYTHON_H
