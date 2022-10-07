#ifndef MULTI_KINECT_WRAPPER_H
#define MULTI_KINECT_WRAPPER_H

#include "DoOnce.h"
#include "MKPPool.h"
#include "MKPRecipient.h"
#include "ThreadPool.h"
#include <cstdlib>

class KinectWrapper;
class MKConfig;

class MultiKinectWrapper : public DoOnce {
public:
    static size_t getNumCameras();
    static void seeConnectedCameras();

    MultiKinectWrapper(MKConfig &config);
    ~MultiKinectWrapper();

    void setRecipient(MKPRecipient *_mkpr);

    KinectWrapper *getKinectWrapper(size_t index);
    MKConfig &getConfig();

    void doOnce();
    std::vector<KinectWrapper *> getKinectWrappers();
    
protected:
    MKPPool _pool;
    MKPRecipient *_mkpr;
    std::vector<KinectWrapper *> _kw;
};

#endif