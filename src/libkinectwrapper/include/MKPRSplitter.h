#ifndef MKPR_SPLITTER_H
#define MKPR_SPLITTER_H

#include "MKPRecipient.h"

#include <vector>

class MKPRSplitter : public MKPRecipient {
public:
    void addRecipient(MKPRecipient *r);
    void receiveFrame(MultiKinectPacket &mkp);

protected:
    std::vector<MKPRecipient *> _v;
};

#endif