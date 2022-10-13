#ifndef MKPR_COMPUTE_GOAL_H
#define MKPR_COMPUTE_GOAL_H

#include "MKPRecipient.h"
#include <k4abt.hpp>

class MKPRecipient;
class MultiKinectWrapper;

class MKPComputeGoal : public MKPRecipient {

protected:
    MultiKinectWrapper &_mkw;
    std::vector<MKPRecipient *> _r;
    k4abt::frame body_frame;  

    struct Point {             
        float x;
        float y;
        float z;   
    }; 

    struct Quaternion {  
        float w;           
        float x;
        float y;
        float z;    
    };  
    
    struct GoalPose {             
        struct Point position;         
        struct Quaternion orientation;   
    };     

    struct GoalPose goalPose;
    
    void computeGoal();
    void scanLine();
    void polyfit(const std::vector<double> &t, const std::vector<double> &v, std::vector<double> &coeff, int order);

    float avgDistance(k4abt::frame bodyFrame);
    std::vector<float> getFittedLine(k4abt::frame bodyFrame);
    k4abt_skeleton_t getLastPerson(k4abt::frame bodyFrame);
    std::vector<k4abt_skeleton_t> getEndPeople(k4abt::frame bodyFrame);
    k4abt_skeleton_t getRightmostPerson(std::vector<k4abt_skeleton_t> people);
    float* calculateJointPos(k4abt_skeleton_t skeleton, k4abt_joint_id_t joint);

    float averageDistance;
    std::vector<float> fittedLine;
    k4abt_skeleton_t lastPerson;

    // for debugging
    float position [3];
    float orientation [4];

public:
    MKPComputeGoal(MultiKinectWrapper &mkw);
    ~MKPComputeGoal();

    void addRecipient(MKPRecipient *r);
    void receiveFrame(MultiKinectPacket &mkp);

    GoalPose getGoal();
    
    // for debugging
    float* getPosition();
    float* getOrientation(); 

};

#endif