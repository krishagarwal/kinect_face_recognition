#ifndef MKPR_RECORD_H
#define MKPR_RECORD_H

#include "MKPRecipient.h"

#include <k4a/k4a.h>
#include <k4arecord/record.hpp>
#include <string>
#include <vector>

class MKPRRecord : public MKPRecipient {
public:
    MKPRRecord(const std::string& outPath);
    void receiveFrame(MultiKinectPacket& mkp);
    void close();
protected:
    void initializeHandles(MultiKinectPacket& mkp);

    std::vector<k4a::record> _recordingHandles;
    bool _recordingHandlesInitialized;
    std::string _outPath;
};

#endif