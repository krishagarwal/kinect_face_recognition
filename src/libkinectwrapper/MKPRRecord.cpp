#include "include/MKPRRecord.h"
#include "include/MultiKinectPacket.h"
#include "include/KinectWrapper.h"
#include <iostream>
#include <sys/stat.h>

using std::cout;
using std::endl;

MKPRRecord::MKPRRecord(const std::string& outPath)
    : _recordingHandlesInitialized(false), _outPath(outPath)
{
    mkdir(outPath.c_str(), 0777);
}

void MKPRRecord::receiveFrame(MultiKinectPacket& mkp)
{
    initializeHandles(mkp); // Skips if already initialized
    for(int i = 0; i < _recordingHandles.size(); i++){
        _recordingHandles[i].write_capture(mkp[i].getCapture());
    }
}

void MKPRRecord::close()
{
    for (int i = 0; i < _recordingHandles.size(); i++) {
        cout << "closing i: " << i << endl;
        _recordingHandles[i].close();
    }
}

void MKPRRecord::initializeHandles(MultiKinectPacket& mkp)
{
    if(_recordingHandlesInitialized){
        return;
    }

    // Initialize recording handle for each KinectWrapper
    std::vector<KinectWrapper*> wrappers = mkp.getKinectWrappers();
    for (int i = 0; i < wrappers.size(); i++) {
        std::string imagePath = _outPath + "/recording_" + std::to_string(i) + ".mkv";
        _recordingHandles.push_back(wrappers[i]->getRecordingHandle(imagePath));
        // Write header immediately
        _recordingHandles[i].write_header();
    }

    _recordingHandlesInitialized = true;
}