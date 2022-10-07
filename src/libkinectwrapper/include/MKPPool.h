#ifndef MK_POOL_H
#define MK_POOL_H

#include <set>
#include <vector>
#include "ThreadPool.h"

class MKConfig;
class MultiKinectPacket;

class MKPPool {
public:
    MKPPool(MKConfig &config);
    ~MKPPool();

    MKConfig &getConfig();

    ThreadPoolQueue _threadPool;
protected:
    MKConfig &_config;
    std::vector<MultiKinectPacket *> _free;
    std::set<MultiKinectPacket *> _inUse;

};

#endif