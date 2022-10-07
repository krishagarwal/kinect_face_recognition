#ifndef MKP_RECIPIENT_H
#define MKP_RECIPIENT_H

class MultiKinectPacket;

class MKPRecipient {
public:
    virtual void receiveFrame(MultiKinectPacket &mkp) = 0;
};

#endif