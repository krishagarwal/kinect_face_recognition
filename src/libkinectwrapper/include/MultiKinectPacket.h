#ifndef MULTI_KINECT_PACKET_H
#define MULTI_KINECT_PACKET_H

#include <Eigen/Eigen>
#include <k4a/k4a.h>
#include <k4abt.hpp>

#include <opencv2/opencv.hpp>

#include <map>
#include <string>
#include <future>

#include <cstdlib>
#include "KinectWrapper.h"
#include "KinectPacket.h"
#include "profiling.h"

class MKConfig;
class MKPPool;
class KinectWrapper;

class MultiKinectPacket {
public:
    MultiKinectPacket(MKPPool &pool, std::vector<KinectWrapper*> wrappers);

    /* accessors */
    MKConfig &getConfig();
    MKPPool &getPool();
    std::vector<KinectWrapper*> getKinectWrappers();

    KinectPacket &operator[](size_t i);

protected:
    std::vector<KinectWrapper*> _wrappers;
    MKPPool &_pool;
    std::vector<KinectPacket> _packets;

public:
    std::vector<k4abt_body_t> solvedBodies;
    std::map<size_t, std::map<uint32_t, uint32_t>> _localToGlobalBodyIds;

    void reset();
};

#endif