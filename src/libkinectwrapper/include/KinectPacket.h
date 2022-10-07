#ifndef KINECT_PACKET_H
#define KINECT_PACKET_H

#include <Eigen/Eigen>
#include <k4a/k4a.h>
#include <k4abt.hpp>

#include <opencv2/opencv.hpp>

#include <map>
#include <string>

#include <cstdlib>
#include "KinectWrapper.h"
#include "profiling.h"

class MKConfig;
class MKPPool;
class KinectWrapper;

class KinectPacket {
public:
    KinectPacket(MKPPool *pool, KinectWrapper *wrapper);
    ~KinectPacket();

    /* standard MKP functionality */
    k4a::capture getCapture();
    void setCapture(k4a::capture capture);
    cv::Mat getRGBColor();
    cv::Mat getRGBColorPreviewScale();
    cv::Mat getRGBDepth();
    k4a::image getXYZDepth();

    k4abt::frame getBodyFrame();
    void setBodyFrame(k4abt::frame frame);

    /* accessors */
    MKConfig &getConfig();
    KinectWrapper *getKinectWrapper();

    std::map<std::string, Eigen::MatrixXd> cornerDetections;

protected:
    KinectWrapper *_wrapper;
    MKPPool *_pool;

private:
    k4a::image getBGRAColorKinect();

    k4a::capture _capture;
    k4a::image bgraColor;
    bool bgraColorReady = false;
    k4a::image xyzPointCloud;
    bool xyzPointCloudReady = false;
    k4abt::frame _bodyTracks;

    cv::Mat rgbColor;
    bool rgbColorReady = false;
    cv::Mat rgbDepth;
    bool rgbDepthReady = false;
    cv::Mat rgbColorPreview;
    bool rgbColorPreviewReady = false;
};

#endif