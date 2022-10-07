#ifndef MK_CONFIG_H
#define MK_CONFIG_H

#include <k4a/k4a.hpp>

class MKConfig {
public:
    MKConfig(size_t nCameras, size_t maxPeople = 0);
    size_t _nCameras;
    size_t _maxPeople;
    
    k4a_device_configuration_t _config;
    size_t _rowsC, _colsC;

    std::string recordFolder;

    std::string getRecordFile(size_t camera);
};

#endif