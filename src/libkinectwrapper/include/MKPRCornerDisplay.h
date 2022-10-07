#ifndef MKPR_CORNER_DISPLAY_H
#define MKPR_CORNER_DISPLAY_H

#include "MKPRecipient.h"

class MKPRCornerDisplay : public MKPRecipient {
public:
    // TODO add param for which corners to draw
    void receiveFrame(MultiKinectPacket &mkp);
};

#endif