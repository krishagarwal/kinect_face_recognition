#include "include/MKPRMergeBodyTracks.h"
#include "include/MKConfig.h"
#include "include/BodyTrackConstants.h"
#include <array>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <iostream>

class BodyTrackMergeFunctor {
private:
    k4abt_skeleton_t observedSkeleton;
public:
    BodyTrackMergeFunctor(const k4abt_skeleton_t skeleton)
        : observedSkeleton(skeleton)
    {
    }

    // Arguments:
    // basePos[3]: position of the body origin (pelvis) in this frame
    // lengths[K4ABT_JOINT_COUNT]: the distance of each coordinate relative to its parent, constant for all frames;
    //   the body origin is ignored
    // angles[K4ABT_JOINT_COUNT * 3]: angle of each joint within this frame, in Rodrigues notation; the body origin is ignored
    //   (NOTE: these do not map to the rotations given by the Kinect system)
    // residuals[K4ABT_JOINT_COUNT * 3]
    template <typename T>
    bool operator()(const T* basePos, const T* lengths, const T* angles, T* residuals) const
    {
        residuals[0] = basePos[0] - T(observedSkeleton.joints[0].position.xyz.x);
        residuals[1] = basePos[1] - T(observedSkeleton.joints[0].position.xyz.y);
        residuals[2] = basePos[2] - T(observedSkeleton.joints[0].position.xyz.z);
        T points[K4ABT_JOINT_COUNT][3] = {{basePos[0], basePos[1], basePos[2]}};
        for (size_t i = 1; i < K4ABT_JOINT_COUNT; i++) {
            T extent[] = {lengths[i], T(0), T(0)};
            T currPoint[3] = {T(0), T(0), T(0)};
            // std::cout << currPoint[0] << " " << currPoint[1] << " " << currPoint[2] << std::endl;
            ceres::AngleAxisRotatePoint(angles + 3 * i, extent, currPoint);
            currPoint[0] += points[JOINT_PARENTS[i]][0];
            currPoint[1] += points[JOINT_PARENTS[i]][1];
            currPoint[2] += points[JOINT_PARENTS[i]][2];

            residuals[i * 3] = currPoint[0] - T(observedSkeleton.joints[i].position.xyz.x);
            residuals[i * 3 + 1] = currPoint[1] - T(observedSkeleton.joints[i].position.xyz.y);
            residuals[i * 3 + 2] = currPoint[2] - T(observedSkeleton.joints[i].position.xyz.z);

            points[i][0] = currPoint[0];
            points[i][1] = currPoint[1];
            points[i][2] = currPoint[2];
        }
        return true;
    }
};

BodyTrackData::BodyTrackData() {}

void BodyTrackData::addSkeleton(frame_id frame, k4abt_skeleton_t skeleton) {
    if (!measuredSkeletonsByFrame.count(frame)) {
        measuredSkeletonsByFrame[frame] = std::vector<k4abt_skeleton_t>();
    }
    measuredSkeletonsByFrame[frame].push_back(skeleton);
    if (measuredSkeletonsByFrame.size() == 1) {
        for (int i = 0; i < K4ABT_JOINT_COUNT; i++) {
            double dx = skeleton.joints[i].position.xyz.x - skeleton.joints[JOINT_PARENTS[i]].position.xyz.x;
            double dy = skeleton.joints[i].position.xyz.y - skeleton.joints[JOINT_PARENTS[i]].position.xyz.y;
            double dz = skeleton.joints[i].position.xyz.z - skeleton.joints[JOINT_PARENTS[i]].position.xyz.z;
            lengths[i] = sqrt(dx * dx + dy * dy + dz * dz);
        }
    }
}

struct BodyTrackFrameData {
    double basePos[3];
    double angles[K4ABT_JOINT_COUNT * 3];

    BodyTrackFrameData(k4abt_skeleton_t skeleton) {
        basePos[0] = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.x;
        basePos[1] = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.y;
        basePos[2] = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.z;
        
        for (size_t i = 1; i < K4ABT_JOINT_COUNT; i++) {
            k4a_float3_t &self = skeleton.joints[i].position;
            k4a_float3_t &parent = skeleton.joints[JOINT_PARENTS[i]].position;
            Eigen::AngleAxisd rot(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1, 0, 0),
                Eigen::Vector3d(self.xyz.x - parent.xyz.x, self.xyz.y - parent.xyz.y, self.xyz.z - parent.xyz.z)));
            Eigen::Vector3d rodrigues = rot.axis() * rot.angle();

            angles[i * 3] = rodrigues.x();
            angles[i * 3 + 1] = rodrigues.y();
            angles[i * 3 + 2] = rodrigues.z();

            if (isnan(angles[i * 3]) || isnan(angles[i * 3 + 1]) || isnan(angles[i * 3 + 2])) {
                abort();
            }
        }
    }
};

void BodyTrackData::solve() {
    ceres::Problem problem;

    std::map<frame_id, BodyTrackFrameData *> allFrameData;
    for (auto it = measuredSkeletonsByFrame.begin(); it != measuredSkeletonsByFrame.end(); it++) {
        BodyTrackFrameData *data = new BodyTrackFrameData(it->second[0]);
        allFrameData[it->first] = data;
        for (size_t skeleton = 0; skeleton < it->second.size(); skeleton++) {
            ceres::CostFunction *func = new ceres::AutoDiffCostFunction<BodyTrackMergeFunctor, K4ABT_JOINT_COUNT * 3, 3, K4ABT_JOINT_COUNT, K4ABT_JOINT_COUNT * 3>
                (new BodyTrackMergeFunctor(it->second[skeleton]));
            problem.AddResidualBlock(func, nullptr, (double *)data->basePos,
                (double *)lengths, (double *)data->angles);
        }
    }
    

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = DENSE_NORMAL_CHOLESKY;
    // options.function_tolerance = 1e-16;
    options.parameter_tolerance = 1e-18;
    options.gradient_tolerance = 1e-18;
    options.function_tolerance = 1e-18;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 12;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    generateSkeletons(allFrameData);
    solved = true;
}

void BodyTrackData::generateSkeletons(std::map<frame_id, BodyTrackFrameData *>& data){
    // TODO "should" also generate orientations - laborious since weird flips in joint definitions
    // TODO should also generate confidence levels. Maybe just max of all seen joints in each frame?
    // TODO needs visualization testing
    for(std::pair<frame_id, BodyTrackFrameData *> pair: data){
        BodyTrackFrameData *frame = pair.second;

        k4abt_skeleton_t out;
        out.joints[0].position.xyz.x = frame->basePos[0];
        out.joints[0].position.xyz.y = frame->basePos[1];
        out.joints[0].position.xyz.z = frame->basePos[2];

        for (int i = 1; i < K4ABT_JOINT_COUNT; i++) {
            Eigen::Vector3d point(lengths[i], 0, 0);

            Eigen::Vector3d rotVec(frame->angles[i * 3], frame->angles[i * 3 + 1], frame->angles[i * 3 + 2]);
            Eigen::AngleAxisd rot(rotVec.norm(), rotVec.normalized());

            // Rotate joint to correct position, add parent frame (global coords)
            Eigen::Vector3d pointOut = rot * point;
            pointOut[0] += out.joints[JOINT_PARENTS[i]].position.xyz.x;
            pointOut[1] += out.joints[JOINT_PARENTS[i]].position.xyz.y;
            pointOut[2] += out.joints[JOINT_PARENTS[i]].position.xyz.z;

            out.joints[i].position.xyz.x = pointOut[0];
            out.joints[i].position.xyz.y = pointOut[1];
            out.joints[i].position.xyz.z = pointOut[2];
        }
        solvedSkeletons[pair.first] = (out);
    }

    if(solvedSkeletons.size() != measuredSkeletonsByFrame.size()){
        std::cout << "solved size: " << solvedSkeletons.size() << " measured frames: " << measuredSkeletonsByFrame.size() << std::endl;
        // abort();
    }
}

MKPRMergeBodyTracks::MKPRMergeBodyTracks(const std::vector<Eigen::MatrixXd>& transforms):
    rigidTransformations(transforms), lastTimestamp(frame_id::min()) {
}

k4abt_skeleton_t transformSkeleton(Eigen::MatrixXd rigidTrans, k4abt_skeleton_t in) {
    k4abt_skeleton_t out;
    for (int jointN = 0; jointN < K4ABT_JOINT_COUNT; jointN++) {
        k4abt_joint_t &jointIn = in.joints[jointN];
        k4abt_joint_t &jointOut = out.joints[jointN];
        jointOut.confidence_level = jointIn.confidence_level;

        Eigen::MatrixXd point(4, 1);
        point(0, 0) = jointIn.position.xyz.x;
        point(1, 0) = jointIn.position.xyz.y;
        point(2, 0) = jointIn.position.xyz.z;
        point(3, 0) = 1;

        Eigen::MatrixXd pointOut = rigidTrans.inverse() * point;
        jointOut.position.xyz.x = (float) (pointOut(0, 0) / pointOut(3, 0));
        jointOut.position.xyz.y = (float) (pointOut(1, 0) / pointOut(3, 0));
        jointOut.position.xyz.z = (float) (pointOut(2, 0) / pointOut(3, 0));

        Eigen::Quaterniond rotIn(jointIn.orientation.wxyz.w, jointIn.orientation.wxyz.x,
            jointIn.orientation.wxyz.y, jointIn.orientation.wxyz.z);
        Eigen::Quaterniond rotTrans(Eigen::Matrix3d(rigidTrans.block(0, 0, 3, 3)));
        Eigen::Quaterniond rotOut = rotTrans.inverse() * rotIn;
        jointOut.orientation.wxyz.w = (float) rotOut.w();
        jointOut.orientation.wxyz.x = (float) rotOut.x();
        jointOut.orientation.wxyz.y = (float) rotOut.y();
        jointOut.orientation.wxyz.z = (float) rotOut.z();

        jointOut.confidence_level = jointIn.confidence_level;
    }
    return out;
}

// Maximum distance (in millimeters) between two body measurements for them
// to count as the same person. Can be an overestimate; the closest track
// will be used by default.
#define BODY_TRACK_SHIFT_MM 600

void MKPRMergeBodyTracks::receiveFrame(MultiKinectPacket &mkp) {
    frame_id frameID = mkp[0].getCapture().get_color_image().get_device_timestamp();
    // For less-hacky recording playback
    frame_id prevFrameID = lastTimestamp;
    if (frameID < lastTimestamp) { // Recording is looping back, solve
        if (!allSolved) {
            solve();
            allSolved = true;
        }
    } else {
        lastTimestamp = frameID;
    }
    bool hasBody = false;
    for (size_t cam = 0; cam < mkp.getConfig()._nCameras; cam++) {
        const k4abt::frame &frame = mkp[cam].getBodyFrame();

        for (size_t bodyN = 0; bodyN < frame.get_num_bodies(); bodyN++) {
            k4abt_body_t body = frame.get_body(bodyN);

            uint32_t lastTrack = uint32_t(bodyTracks.size());
            double lastTrackDistSq = BODY_TRACK_SHIFT_MM * BODY_TRACK_SHIFT_MM;

            k4abt_skeleton_t globalSkel = transformSkeleton(rigidTransformations[cam], body.skeleton);

            for (auto &existingTrack : bodyTracks) {
                k4abt_skeleton_t *prevSkeleton = nullptr;
                if (existingTrack.second.measuredSkeletonsByFrame.count(frameID)) {
                    prevSkeleton = &existingTrack.second.measuredSkeletonsByFrame[frameID][0];
                } else if (existingTrack.second.measuredSkeletonsByFrame.count(prevFrameID)) {
                    prevSkeleton = &existingTrack.second.measuredSkeletonsByFrame[prevFrameID][0];
                } else {
                    continue;
                }
                k4a_float3_t &prevPos = prevSkeleton->joints[K4ABT_JOINT_PELVIS].position;
                k4a_float3_t &currPos = globalSkel.joints[K4ABT_JOINT_PELVIS].position;
                double distSq = (prevPos.xyz.x - currPos.xyz.x) * (prevPos.xyz.x - currPos.xyz.x)
                    + (prevPos.xyz.y - currPos.xyz.y) * (prevPos.xyz.y - currPos.xyz.y)
                    + (prevPos.xyz.z - currPos.xyz.z) * (prevPos.xyz.z - currPos.xyz.z);
                if (distSq < lastTrackDistSq) {
                    lastTrack = existingTrack.first;
                    lastTrackDistSq = distSq;
                }
            }
            
            mkp._localToGlobalBodyIds[cam][body.id] = lastTrack;
            if (!bodyTracks[lastTrack].solved) {
                bodyTracks[lastTrack].addSkeleton(frameID, globalSkel);
            }
        }
    }
    if (allSolved) {
        for (auto it = bodyTracks.begin(); it != bodyTracks.end(); it++) {
            if (it->second.solvedSkeletons.count(frameID)) {
                k4abt_body_t solvedBody = {
                    .id = it->first,
                    .skeleton = it->second.solvedSkeletons[frameID]
                };
                mkp.solvedBodies.push_back(solvedBody);
                hasBody = true;
            }
        }
    }
}

void MKPRMergeBodyTracks::solve() {
    for (auto it = bodyTracks.begin(); it != bodyTracks.end(); it++) {

        std::cout << "Body " << it->first << std::endl;
        std::cout << "INITIAL ESTIMATE" << std::endl;
        for (size_t i = 0; i < K4ABT_JOINT_COUNT; i++) {
            std::cout << "Bone length J" << JOINT_PARENTS[i] << "-J" << i << ": " << it->second.lengths[i] << std::endl;
        }
        std::cout << std::endl;

        it->second.solve();

        std::cout << "SOLVED" << std::endl;
        for (size_t i = 0; i < K4ABT_JOINT_COUNT; i++) {
            std::cout << "Bone length J" << JOINT_PARENTS[i] << "-J" << i << ": " << it->second.lengths[i] << std::endl;
        }
        std::cout << std::endl;
    }
}
