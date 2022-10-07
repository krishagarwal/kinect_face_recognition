#ifndef MKPR_SAVE_FRAME_H
#define MKPR_SAVE_FRAME_H

#include "MKPRecipient.h"
#include <string>

class MKPRSaveFrame : public MKPRecipient {
public:
    MKPRSaveFrame(std::string path);
    void receiveFrame(MultiKinectPacket &mkp);
    void saveFrame();
protected:
    void writeToFile(MultiKinectPacket& mkp);
    std::string _savePath;
    bool _performSave;
    int _setsCaptured;
};

#endif