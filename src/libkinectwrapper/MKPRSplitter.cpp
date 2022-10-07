#include "include/MKPRSplitter.h"

void MKPRSplitter::addRecipient(MKPRecipient *r) {
    _v.push_back(r);
}

void MKPRSplitter::receiveFrame(MultiKinectPacket &mkp) {
    for(size_t i = 0; i < _v.size(); i++)
        _v[i]->receiveFrame(mkp);
}