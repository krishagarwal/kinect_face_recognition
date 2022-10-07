#ifndef MKPR_MERGE_BODY_TRACKS_H
#define MKPR_MERGE_BODY_TRACKS_H
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <Eigen/Dense>
#include "MKPRecipient.h"
#include "MultiKinectPacket.h"

struct BodyTrackFrameData;

typedef std::chrono::microseconds frame_id;

class BodyTrackData {
public:
    double lengths[K4ABT_JOINT_COUNT];
    std::map<frame_id, std::vector<k4abt_skeleton_t>> measuredSkeletonsByFrame;
    std::map<frame_id, k4abt_skeleton_t> solvedSkeletons;
    bool solved = false;
    BodyTrackData();
    void addSkeleton(frame_id frame, k4abt_skeleton_t skeleton);
    void generateSkeletons(std::map<frame_id, BodyTrackFrameData *>& data);
    void solve();
};

class MKPRMergeBodyTracks: public MKPRecipient{
private:
    std::vector<Eigen::MatrixXd> rigidTransformations;
    std::map<uint32_t, BodyTrackData> bodyTracks;
    frame_id lastTimestamp;
    bool allSolved = false;
public:
    MKPRMergeBodyTracks(const std::vector<Eigen::MatrixXd>& transforms);
    void receiveFrame(MultiKinectPacket &mkp);
    void solve();
};

#endif