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

    int num_real_bodies;
    int num_spoofed;

    //struct Pose goalPose;
    
    void computeGoal();
    void scanLine();
    void polyfit(const std::vector<double> &t, const std::vector<double> &v, int order);

    float avgDistance(k4abt::frame bodyFrame);

    void getFittedLine();

    k4abt_skeleton_t getLastPerson(k4abt::frame bodyFrame);
    std::vector<k4abt_skeleton_t> getEndPeople(k4abt::frame bodyFrame);
    k4abt_skeleton_t getRightmostPerson(std::vector<k4abt_skeleton_t> people);
    void loadBodies();
    void loadSpoofed(float distance);


    float* calculateJointPos(k4abt_skeleton_t skeleton, k4abt_joint_id_t joint);
    float* calculateJointOrient(k4abt_skeleton_t skeleton, k4abt_joint_id_t joint);

    float averageDistance;
    k4abt_skeleton_t lastPerson;

    // for debugging
    float position [3];
    float orientation [4];

public:

    struct Point {             
        float x;
        float y;
        float z;
    }; 

    struct Quaternion {          
        float x;
        float y;
        float z;   
        float w; 
    };  

    struct Pose {             
        Point* position;         
        Quaternion* orientation;   
    };   

    MKPComputeGoal(MultiKinectWrapper &mkw);
    ~MKPComputeGoal();

    void addRecipient(MKPRecipient *r);
    void receiveFrame(MultiKinectPacket &mkp);

    Pose getGoal();
    
    // for debugging
    float* getPosition();
    float* getOrientation(); 
    //Pose** getBodies();
    int getNumBodies();
    // std::vector<Point*> getFittedPoints();
    // Point* fittedPoints[20];
    Pose* bodies[20];  
    void printLine();

    std::vector<double> coeffs;

};

#endif