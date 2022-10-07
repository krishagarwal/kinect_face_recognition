#include "include/MultiKinectWrapper.h"
#include "include/MKConfig.h"
#include "include/MKPRecipient.h"
#include "include/MultiKinectPacket.h"
#include "include/KinectWrapperLive.h"
#include "include/KinectWrapperPlayback.h"
#include <iostream>

using namespace k4a;
using namespace std;

size_t MultiKinectWrapper::getNumCameras() {
    return device::get_installed_count();
}

void MultiKinectWrapper::seeConnectedCameras(){
    // Directly from Azure Kinect SDK Tutorial
    
    uint32_t device_count = device::get_installed_count();
    printf("Found %d connected devices:\n", device_count);

    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        device device = device::open(deviceIndex);
        cout << deviceIndex << ": Device \"" << device.get_serialnum() << "\"" << endl;
        device.close();
    }
}

MultiKinectWrapper::MultiKinectWrapper(MKConfig &config) :
    _pool(config),
    _mkpr(NULL) {
    for (int i = 0;i<config._nCameras;i++){
        KinectWrapper *kw;
        if (config.recordFolder.length() == 0) {
            kw = new KinectWrapperLive(i, config, true );
        } else {
            kw = new KinectWrapperPlayback(config, config.getRecordFile(i), true);
        }
        kw->start();
        _kw.push_back(kw);
        // _kw.push_back(new KinectWrapper(i, config, K4A_WIRED_SYNC_MODE_STANDALONE));
    } 
}

 MultiKinectWrapper::~MultiKinectWrapper() {
    for(size_t i = 0; i < _kw.size(); i++)
    delete _kw[i];
    _kw.clear();
 }

void MultiKinectWrapper::setRecipient(MKPRecipient *mkpr) {
    _mkpr = mkpr;
}

KinectWrapper *MultiKinectWrapper::getKinectWrapper(size_t index) {
    return _kw[index];
}

std::vector<KinectWrapper *> MultiKinectWrapper::getKinectWrappers(){
    return _kw;
}

MKConfig &MultiKinectWrapper::getConfig() {
    return _pool.getConfig();
}

void MultiKinectWrapper::doOnce() {
    // cout << "MultiKinectWrapper::doOnce():" << endl;
    MultiKinectPacket pkt(_pool, _kw);
    {
    PROFILE_START;
    vector<future<void>> futures;
    for (int i = 0; i < _kw.size(); i++) {
        KinectWrapper &wrapper = *_kw[i];
        KinectPacket &campkt = pkt[i];
        futures.push_back(_pool._threadPool.addJob(packaged_task<void ()>([&wrapper, &campkt] {
            k4a::capture cap = campkt.getCapture();
            wrapper.capture(&cap);
            campkt.setCapture(cap);
        })));
    }
    for (future<void> &f : futures) {
        f.wait();
    }
    PROFILE_END("captures");
    }

    PROFILE_START;
    // cout<<"sending frame"<<endl;
    if(_mkpr != NULL)   _mkpr->receiveFrame(pkt);
    PROFILE_END("all MKPRs");
    PROFILE_INTERVAL("doOnce interval");
}
