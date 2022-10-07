#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include <k4a/k4a.hpp>
#include <k4arecord/record.hpp>
#include <vector>
#include <string>

class KinectWrapper {
public:
    //The sync master must be started last
    virtual void start() = 0;
    virtual void capture(k4a::capture *cap) = 0;
    virtual void setupSync(int32_t color_exposure_usec,int32_t powerline_freq) = 0;
    virtual k4a_device_configuration_t  getConfig() = 0;
    virtual k4a::calibration            getCalibration() = 0;
    virtual std::vector<uint8_t>        getRawCalibration() = 0;
    virtual k4a::record                 getRecordingHandle(const std::string&) = 0;
    k4a::transformation transformation;
};

#endif
