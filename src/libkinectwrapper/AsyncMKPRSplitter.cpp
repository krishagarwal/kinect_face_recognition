#include "include/AsyncMKPRSplitter.h"
#include "include/MultiKinectPacket.h"

AsyncMKPRSplitter::AsyncMKPRSplitter() {}
AsyncMKPRSplitter::~AsyncMKPRSplitter() {
    for (auto &q : queues) {
        q->shutdown();
    }
    for (auto &t : threads) {
        t->join();
    }
}

void AsyncMKPRSplitter::addRecipient(MKPRecipient *r) {
    int i = _v.size();
    _v.push_back(r);
    queues.push_back(std::make_unique<BlockingQueue<MultiKinectPacket *>>(2));

    threads.push_back(std::make_unique<std::thread>([this, i]() {
        MultiKinectPacket *mkp;
        while (queues[i]->pop(mkp)) {
            _v[i]->receiveFrame(*mkp);

            if (i + 1 < _v.size()) {
                queues[i + 1]->push(mkp);
            } else {
                delete mkp;
            }
        }
    }));
}

void AsyncMKPRSplitter::receiveFrame(MultiKinectPacket &mkp) {
    queues[0]->push(new MultiKinectPacket(mkp));
}
