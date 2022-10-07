#include "include/MKConfig.h"

MKConfig::MKConfig(size_t nCameras, size_t maxPeople) : _nCameras(nCameras), _maxPeople(maxPeople), _config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL) {
    _config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    _config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    _config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    _config.synchronized_images_only = true;
    _config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    switch(_config.color_resolution) {
        case K4A_COLOR_RESOLUTION_2160P:
            _rowsC = 2160;
            _colsC = 3840;
            break;
        default:
            abort();
    }
}

std::string MKConfig::getRecordFile(size_t num) {
    if (recordFolder.length() == 0) {
        return "";
    }
    return recordFolder + "/recording_" + std::to_string(num) + ".mkv";
}
