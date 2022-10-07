#include "MKPRecipient.h"
#include "KinectPacket.h"

class AsyncMKPRecipient: public MKPRecipient {
public:
    void receiveFrame(MultiKinectPacket &packet);

    virtual void beforeAll(MultiKinectPacket &packet) {}
    virtual void handleCamera(KinectPacket &packet, size_t camera) = 0;
    virtual void afterAll(MultiKinectPacket &packet) {}
};