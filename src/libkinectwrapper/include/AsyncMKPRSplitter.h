#ifndef MKPR_SPLITTER_H
#define MKPR_SPLITTER_H

#include "MKPRecipient.h"

#include <vector>

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

template <typename T> class BlockingQueue {
private:
    std::queue<T> queue;
    std::mutex mutex;
    std::condition_variable cv_out;
    std::condition_variable cv_in;
    bool _shutdown = false;
    size_t capacity;
public:
    BlockingQueue(size_t capacity): capacity(capacity) {}
    BlockingQueue(const BlockingQueue &) = delete;

    bool push(T pkt) {
        std::unique_lock<std::mutex> lock(mutex);
        if (_shutdown) {
            return false;
        }
        while (queue.size() >= capacity) {
            cv_in.wait(lock);
            if (_shutdown) {
                return false;
            }
        }
        queue.push(pkt);
        cv_out.notify_one();
        return true;
    }

    bool pop(T &ret) {
        std::unique_lock<std::mutex> lock(mutex);
        while (queue.empty()) {
            cv_out.wait(lock);
            if (_shutdown) {
                return false;
            }
        }
        ret = queue.front();
        queue.pop();
        cv_in.notify_one();
        return true;
    }

    bool tryPop(T &ret) {
        std::unique_lock<std::mutex> lock(mutex);
        if (!queue.empty()) {
            ret = queue.front();
            queue.pop();
            cv_in.notify_one();
            return true;
        }
        return false;
    }

    size_t size() {
        std::unique_lock<std::mutex> lock(mutex);
        return queue.size();
    }

    void shutdown() {
        std::unique_lock<std::mutex> lock(mutex);
        _shutdown = true;
        cv_out.notify_all();
        cv_in.notify_all();
    }
};

class AsyncMKPRSplitter : public MKPRecipient {
public:
    void addRecipient(MKPRecipient *r);
    void receiveFrame(MultiKinectPacket &mkp);

    AsyncMKPRSplitter(const AsyncMKPRSplitter &) = delete;
    AsyncMKPRSplitter();
    ~AsyncMKPRSplitter();
private:
    std::vector<MKPRecipient *> _v;
    std::vector<std::unique_ptr<BlockingQueue<MultiKinectPacket *>>> queues;
    std::vector<std::unique_ptr<std::thread>> threads;
    void threadHandler(size_t i);
};

#endif