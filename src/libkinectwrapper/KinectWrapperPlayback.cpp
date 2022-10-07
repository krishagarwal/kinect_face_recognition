#include "include/KinectWrapperPlayback.h"
#include "include/MKConfig.h"
#include <k4arecord/playback.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

using namespace std;

KinectWrapperPlayback::KinectWrapperPlayback(MKConfig config,string filename, bool syncMode) :
    _device(NULL), _config(config._config) {

    _device = k4a::playback::open(filename.c_str());
    _sensor_calibration = _device.get_calibration();
    
    if (syncMode) {
        // Standalone
        _config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    } else if (false) { // TODO
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

KinectWrapperPlayback::~KinectWrapperPlayback() {
    _device.close();
}
void KinectWrapperPlayback::setupSync(int32_t color_exposure_usec, int32_t powerline_freq){
}
k4a_device_configuration_t KinectWrapperPlayback::getConfig(){
    return _config;
}

k4a::calibration KinectWrapperPlayback::getCalibration(){
    return _sensor_calibration;
}


std::vector<uint8_t> KinectWrapperPlayback::getRawCalibration(){
    return _device.get_raw_calibration();
}

k4a::record KinectWrapperPlayback::getRecordingHandle(const std::string& path){
    // TODO Although the device can be null, it discards some important info like the calibration.
    cerr << "Cannot re-record a playback at this time" << endl;
    abort();
}

void KinectWrapperPlayback::start() {
}

void KinectWrapperPlayback::capture(k4a::capture *cap) {
    if (!_device.get_next_capture(cap)) {
        _device.seek_timestamp(std::chrono::milliseconds(0), K4A_PLAYBACK_SEEK_BEGIN);
        if (!_device.get_next_capture(cap)) {
            throw "Recording file is empty";
        }
    }

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::microseconds frame_write_time = cap->get_color_image().get_device_timestamp();
    if (last_frame_read_time.time_since_epoch().count() != 0 && frame_write_time > last_frame_write_time) {
        std::this_thread::sleep_until(last_frame_read_time + (frame_write_time - last_frame_write_time));
    }
    last_frame_read_time = now;
    last_frame_write_time = frame_write_time;
}
