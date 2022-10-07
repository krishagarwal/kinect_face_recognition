#ifndef MKP_PERSON_TRACKER_H
#define MKP_PERSON_TRACKER_H

#include "MKPRecipient.h"

#include <k4abt.hpp>

class MKPRecipient;
class MultiKinectWrapper;

class MKPPersonTracker : public MKPRecipient {
public:
    MKPPersonTracker(MultiKinectWrapper &mkw);
    ~MKPPersonTracker();

    void addRecipient(MKPRecipient *r);

    void receiveFrame(MultiKinectPacket &mkp);
    
protected:
    MultiKinectWrapper &_mkw;
    std::vector<MKPRecipient *> _r;
    
    k4abt_tracker_configuration_t _tracker_config;
    k4abt::tracker *_tracker;
};

#endif