#include <future>
#include <vector>
#include "include/AsyncMKPRecipient.h"
#include "include/MultiKinectPacket.h"
#include "include/MKPPool.h"
#include "include/KinectWrapper.h" 

using namespace std;

void AsyncMKPRecipient::receiveFrame(MultiKinectPacket &packet) {
    beforeAll(packet);
    vector<future<void>> futures;
    for (int i = 0; i < packet.getKinectWrappers().size(); i++) {
        futures.push_back(packet.getPool()._threadPool.addJob(packaged_task<void ()>([this, &packet, i] {
            handleCamera(packet[i], i);
        })));
    }
    for (future<void> &f : futures) {
        f.wait();
    }
    afterAll(packet);
}
