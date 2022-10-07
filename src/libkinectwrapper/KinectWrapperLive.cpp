#include "include/KinectWrapperLive.h"
#include "include/MKConfig.h"
#include <k4arecord/playback.h>
#include <cstdio>
#include <cstdlib>

using namespace std;

KinectWrapperLive::KinectWrapperLive(uint8_t deviceIndex, MKConfig config, bool syncMode) :
    _device(NULL), _config(config._config) {

    _device = k4a::device::open(deviceIndex);
    _sensor_calibration = _device.get_calibration(_config.depth_mode, _config.color_resolution);
    
    if (syncMode) {
        // Standalone
        _config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    } else if (_device.is_sync_out_connected() && !_device.is_sync_in_connected()) {
        _config.wired_sync_mode =  K4A_WIRED_SYNC_MODE_MASTER;
        uint32_t MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC = 160;
        _config.depth_delay_off_color_usec = -static_cast<int32_t>(MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC / 2);
        _config.synchronized_images_only = true;
    } else {
        _config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
        _config.subordinate_delay_off_master_usec = 0;
        _config.synchronized_images_only = true;
    }

    // Recipient specific config
    //JWH NEED THIS _kfr.getDevice(_device, _config);
    //JWH NEED THIS _kfr.getCalibration(sensor_calibration);
    transformation = k4a::transformation(_sensor_calibration);
}

KinectWrapperLive::~KinectWrapperLive() {
    _device.stop_cameras();
    _device.close();
}
void KinectWrapperLive::setupSync(int32_t color_exposure_usec, int32_t powerline_freq){
    _device.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, color_exposure_usec);
    // _device.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY,K4A_COLOR_CONTROL_MODE_MANUAL,powerline_freq);
}

k4a_device_configuration_t KinectWrapperLive::getConfig(){
    return _config;
}

k4a::calibration KinectWrapperLive::getCalibration(){
    return _sensor_calibration;
}


std::vector<uint8_t> KinectWrapperLive::getRawCalibration(){
    return _device.get_raw_calibration();
}

k4a::record KinectWrapperLive::getRecordingHandle(const std::string& path){
    return k4a::record::create(path.c_str(), _device, _config);
}

void KinectWrapperLive::start() {
    _device.start_cameras(&_config);
}

void KinectWrapperLive::capture(k4a::capture *cap) {
    _device.get_capture(cap, std::chrono::milliseconds(K4A_WAIT_INFINITE));
}