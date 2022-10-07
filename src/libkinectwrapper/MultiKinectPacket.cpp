#include "include/MultiKinectPacket.h"
#include "include/MKConfig.h"
#include "include/MKPPool.h"
#include "include/KinectPacket.h"

MultiKinectPacket::MultiKinectPacket(MKPPool &pool, std::vector<KinectWrapper *> wrappers) :
    _pool(pool), _wrappers(wrappers) {

    size_t n_cameras =  _pool.getConfig()._nCameras;
    for (size_t i = 0; i < n_cameras; i++) {
        _packets.push_back(KinectPacket(&pool, wrappers[i]));
    }
}

std::vector<KinectWrapper*> MultiKinectPacket::getKinectWrappers() {
    return _wrappers;
}

MKPPool &MultiKinectPacket::getPool() {
    return _pool;
}

MKConfig &MultiKinectPacket::getConfig() {
    return _pool.getConfig();
}

KinectPacket &MultiKinectPacket::operator[](size_t i) {
    return _packets[i];
}

void MultiKinectPacket::reset() {
    _localToGlobalBodyIds.clear();
    solvedBodies.clear();
}
