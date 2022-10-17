#include "include/KinectWrapper.h"
#include "include/MKConfig.h"
#include "include/MultiKinectPacket.h"
#include "include/MKPRecipient.h"
#include "include/MultiKinectWrapper.h"
#include "include/MKPComputeGoal.h"
#include <cstdlib>

// for polyfit
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <Eigen/QR>

MKPComputeGoal::MKPComputeGoal(MultiKinectWrapper &mkw) : _mkw(mkw) {
    num_spoofed = 0;
    num_real_bodies = 0;
    float dist = 3;

    loadSpoofed(dist);
}

MKPComputeGoal::~MKPComputeGoal() {}

void MKPComputeGoal::addRecipient(MKPRecipient *r) {
    _r.push_back(r);
}

/* get the body frame from the packet and fill out the position and orientation arrays with the goal */
void MKPComputeGoal::receiveFrame(MultiKinectPacket &mkp) {
    PROFILE_START;
    
    // set body frame object
    body_frame = mkp[0].getBodyFrame();
    if (body_frame) {
        loadBodies();
    }
    scanLine();
    //computeGoal();

    PROFILE_END("MKPComputeGoal.receiveFrame");
}

void MKPComputeGoal::loadBodies() {
    num_real_bodies = 0;
    for (int i = 0; i < body_frame.get_num_bodies(); i++) {
        k4abt_skeleton_t skeleton = body_frame.get_body_skeleton(i);        
        k4abt_joint_id_t joint = K4ABT_JOINT_PELVIS;

        Point* pt = new Point;
        pt->x = skeleton.joints[joint].position.xyz.x / 1000.0 * 3.281;
        pt->y = skeleton.joints[joint].position.xyz.y / 1000.0 * 3.281;
        pt->z = skeleton.joints[joint].position.xyz.z / 1000.0 * 3.281;


        Quaternion* quat = new Quaternion;
        quat->w = skeleton.joints[joint].orientation.wxyz.w;
        quat->x = skeleton.joints[joint].orientation.wxyz.x;
        quat->y = skeleton.joints[joint].orientation.wxyz.y;
        quat->z = skeleton.joints[joint].orientation.wxyz.z;

        Pose* pose = new Pose;
        pose->position = pt;
        pose->orientation = quat;

        bodies[num_spoofed + num_real_bodies] = pose;
        num_real_bodies++;
    }
}

void MKPComputeGoal::loadSpoofed(float distance) {
    Point start_pos = {-4, 0, 4};
    for  (int i = 0; i < num_spoofed; i++) {

        float x_rand = (rand() % 100)/100.0;
        float z_rand = (rand() % 100)/100.0;

        Point* pt = new Point;
        pt->x = start_pos.x + i*distance + x_rand;
        pt->y = start_pos.y;
        pt->z = start_pos.z + z_rand;

        float testw = 0.691;
        float testx = 0.019;
        float testy = 0.029;
        float testz = -0.722;

        Quaternion* quat = new Quaternion;
        
        quat->w = testw;
        quat->x = testx;
        quat->y = testy;
        quat->z = testz;

        Pose* pose = new Pose;
        pose->position = pt;
        pose->orientation = quat;
        
        bodies[i] = pose;
    }
}

float* MKPComputeGoal::getPosition() {
    return position; 
}

float* MKPComputeGoal::getOrientation() {
    return orientation;
} 

int MKPComputeGoal::getNumBodies() {
    // printf("get num bodies returning: %d\n", num_real_bodies + num_spoofed);
    return num_real_bodies + num_spoofed;
}

/* determine spacing, polyfit, and last person, save in class variables */
void MKPComputeGoal::scanLine() {
    /* 1. determine average spacing. iterate through ine two people at a time, 
    compute avg distance between pelvis joints and save average */
    // averageDistance = avgDistance(body_frame);
    // /* 2. determine the last person in the line */
    // lastPerson = getLastPerson(body_frame);
    /* 3. create the fitted line */
    getFittedLine();
}

/* fills out the position and orientation arrays with the person's position and orientation. */
void MKPComputeGoal::computeGoal() {
   /* from the last person, move avgDistance amount along the fittedLine. fill out class variable goalPose */
}


float MKPComputeGoal::avgDistance(k4abt::frame bodyFrame) {
    float avg = 0.0; 
    /* implement*/
    return avg;
}

float* MKPComputeGoal::calculateJointPos(k4abt_skeleton_t skeleton, k4abt_joint_id_t joint) {
    float position[3];
    position[0] = skeleton.joints[joint].position.xyz.x / 1000.0 * 3.281;
    position[1] = skeleton.joints[joint].position.xyz.y / 1000.0 * 3.281;
    position[2] = skeleton.joints[joint].position.xyz.z / 1000.0 * 3.281;

    printf("Point: {%.3f,%.3f,%.3f}\n", position[0],position[1],position[2]);

    return position;
}

float* MKPComputeGoal::calculateJointOrient(k4abt_skeleton_t skeleton, k4abt_joint_id_t joint) {
    float orientation[4];

    orientation[0] = skeleton.joints[joint].orientation.wxyz.x;
    orientation[1] = skeleton.joints[joint].orientation.wxyz.y;
    orientation[2] = skeleton.joints[joint].orientation.wxyz.z;
    orientation[3] = skeleton.joints[joint].orientation.wxyz.w;

    // printf("Quaternion: {%.3f,%3f,%3f,%3f}\n", orientation[0], orientation[1], orientation[2], orientation[3]);
    // printf("-----\n");

    return orientation;
}

//  calculate the expected position of the last person in line
void MKPComputeGoal::polyfit(const std::vector<double> &t, const std::vector<double> &v, int order)
{
	// Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
	Eigen::MatrixXd T(t.size(), order + 1);
	Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
	Eigen::VectorXd result;

	// check to make sure inputs are correct
	assert(t.size() == v.size());
	assert(t.size() >= order + 1);
	// Populate the matrix
	for(size_t i = 0 ; i < t.size(); ++i)
	{
		for(size_t j = 0; j < order + 1; ++j)
		{
			T(i, j) = pow(t.at(i), j);
		}
	}
	
	// Solve for linear least square fit
	result  = T.householderQr().solve(V);
	coeffs.resize(order+1);
	for (int k = 0; k < order+1; k++)
	{
		coeffs[k] = result[k];
	}

}
 

void MKPComputeGoal::getFittedLine() {
    std::vector<double> xpos;
    std::vector<double> zpos;

    // get fitted line from the bodies array we already loaded
    for (int i = 0; i < num_real_bodies+num_spoofed; i++) {
        xpos.push_back(bodies[i]->position->x);
        zpos.push_back(bodies[i]->position->z);
    }

    // printf("\nnumber of bodies: %d\n", num_real_bodies+num_spoofed);
    // get fitted function for predicting x value based on z value
    polyfit(zpos, xpos, 1);
}

std::vector<k4abt_skeleton_t> MKPComputeGoal::getEndPeople(k4abt::frame bodyFrame) {
    k4abt_skeleton_t lastPerson;

    /* define a threshold value - if there is anyone within this distance, they are considered near*/
    double distance_threshold = 3.0;
    std::vector<k4abt_skeleton_t> end_people;
    
    // Go through each body in line
    for (int i = 0; i < bodyFrame.get_num_bodies(); i++) {
        k4abt_skeleton_t person1 = bodyFrame.get_body_skeleton(i);
        int num_people_within = 0;

        // get current person's x and z pos
        float* person1Pos = calculateJointPos(person1, K4ABT_JOINT_PELVIS);
        double person1x = person1Pos[0];
        double person1z = person1Pos[2];

        // compare current body with all other bodies in frame to check how many are near this body
        for (int j = i+1; j < bodyFrame.get_num_bodies(); j++) {
            k4abt_skeleton_t person2 = bodyFrame.get_body_skeleton(j);

            // get comparing person's x and z pos
            float* person2Pos = calculateJointPos(person2, K4ABT_JOINT_PELVIS);
            double person2x = person1Pos[0];
            double person2z = person1Pos[2];

            // use distance formula to get absolute difference between two people
            double difference = sqrt(pow(person1x-person2x, 2) +  pow(person1z-person2z, 2));

            // if person2 is considered to be near person1, increment num_people_within
            if (difference <= distance_threshold) num_people_within++;

        }

        // if none or only 1 person is near the current person, that means they are at the front or back of line
        if (num_people_within <= 1) end_people.push_back(person1);
    
    }

    return end_people;
}

k4abt_skeleton_t MKPComputeGoal::getRightmostPerson(std::vector<k4abt_skeleton_t> people) {
    k4abt_skeleton_t rightPerson = people.at(0);
    float max_size = calculateJointPos(rightPerson, K4ABT_JOINT_PELVIS)[0];
    for (int i = 0; i < people.size(); i++) {
        if (calculateJointPos(people.at(i), K4ABT_JOINT_PELVIS)[0] > max_size) {
            max_size = calculateJointPos(people.at(i), K4ABT_JOINT_PELVIS)[0];
            rightPerson = people.at(i);
        }
    }
    return rightPerson;
}

k4abt_skeleton_t MKPComputeGoal::getLastPerson(k4abt::frame bodyFrame) {
    
    std::vector<k4abt_skeleton_t> end_people = getEndPeople(bodyFrame);
    int numPeople = end_people.size();

    k4abt_skeleton_t last_person;
    
    // now that we have end people, calculate direction line is facing
    
    if (end_people.size() == 0) {
        printf("error! couldn't find end people!\n");
    } else if (end_people.size() == 1) {
        last_person = end_people.back();
    } else {
        /* choose the person with the higher x value. WILL PROBABLY CHANGE THIS BASED ON LOCATION OF TEST */
        last_person = getRightmostPerson(end_people);
    }
    if (end_people.size() > 2) {
        printf("WARNING: detected more than 2 end people!\n");
    }

    return last_person;
}