#include "include/MKPPool.h"
#include "include/MKConfig.h"

MKPPool::MKPPool(MKConfig &config) : _config(config), _threadPool(config._nCameras) {}

MKPPool::~MKPPool() {}

MKConfig &MKPPool::getConfig() {    return _config; }