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
    num_bodies = 0;
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
        loadBodies(2, 3);
    }
    //scanLine();
    //computeGoal();

    
    PROFILE_END("MKPComputeGoal.receiveFrame");
}

// Pose** MKPComputeGoal::getBodies() {
//     return bodies;
// }

void MKPComputeGoal::loadBodies(int num_spoofed, float distance) {
    num_bodies = 0; //body_frame.get_num_bodies();
    printf("\nnum bodies: %d\n", body_frame.get_num_bodies());
    for (int i = 0; i < body_frame.get_num_bodies(); i++) {
        k4abt_skeleton_t skeleton = body_frame.get_body_skeleton(i);
        // float* position = calculateJointPos(skeleton, K4ABT_JOINT_PELVIS);
        // float* orientation = calculateJointOrient(skeleton, K4ABT_JOINT_PELVIS);
        
        Point pt = {skeleton.joints[joint].position.xyz.x / 1000.0 * 3.281, skeleton.joints[joint].position.xyz.y / 1000.0 * 3.281, skeleton.joints[joint].position.xyz.z / 1000.0 * 3.281};
        k4abt_joint_id_t joint = K4ABT_JOINT_PELVIS;
        // ptr->x = skeleton.joints[joint].position.xyz.x / 1000.0 * 3.281;
        // ptr->y = skeleton.joints[joint].position.xyz.y / 1000.0 * 3.281;
        // ptr->z = skeleton.joints[joint].position.xyz.z / 1000.0 * 3.281;
        printf("\nassigned position.\n");

        // printf("position x in struct: %.3f\n", pt.x);

        Quaternion* quat, q;
        quat = &q;
        quat->w = skeleton.joints[joint].orientation.wxyz.w;
        quat->x = skeleton.joints[joint].orientation.wxyz.x;
        quat->y = skeleton.joints[joint].orientation.wxyz.y;
        quat->z = skeleton.joints[joint].orientation.wxyz.z;
        printf("\nassigned orientation.\n");

        Pose pose;
        pose.position = &pt;
        pose.orientation = quat;

        printf("MKP: body %d pose addr: %p\n", num_bodies, &pose);
        bodies[i] = &pose;
        
        printf("\nadded pose to bodies array.\n");

        printf("MKP: body %d x: %.5f\n", i, bodies[i]->position->x);
        //printf("body 1 pose x: %.3f\n", bodies[1]->position.x);
        num_bodies++;
    }


    Point start_pos = {-5, 0, 5};
    printf("start spoofing\n");
    for  (int i = 0; i < num_spoofed; i++) {
        float x_rand = 0.3; //std::rand()/((RAND_MAX + 1u));
        float z_rand = 0.4; //std::rand()/((RAND_MAX + 1u));
        Point* ptr, pt;
        ptr = &pt;
        ptr->x = start_pos.x + i*distance + x_rand;
        ptr->y = start_pos.y;
        ptr->z = start_pos.z + z_rand;
        
        Quaternion* quat, q;
        quat = &q;
        quat->w = 0.0;
        quat->x = 0.0;
        quat->y = 0.0;
        quat->z = 0.0;

        Pose pose;
        pose.position = ptr;
        pose.orientation = quat;
        
        printf("MKP: body %d pose addr: %p\n", num_bodies, &pose);
        bodies[num_bodies] = &pose;
        printf("MKP: body %d x: %.5f\n", num_bodies, bodies[num_bodies]->position->x);
        num_bodies++;
    }
    //num_bodies += num_spoofed;
    printf("end spoofing\n");
    printf("MKP OUTSIDE: body %d x: %.5f\n", 0, bodies[0]->position->x);
    printf("MKP OUTSIDE: body %d x: %.5f\n", 1, bodies[1]->position->x);
    printf("num_bodies: %d\n", num_bodies);
}

float* MKPComputeGoal::getPosition() {
    return position; 
}

float* MKPComputeGoal::getOrientation() {
    return orientation;
} 

int MKPComputeGoal::getNumBodies() {
    return num_bodies;
}

/* determine spacing, polyfit, and last person, save in class variables */
void MKPComputeGoal::scanLine() {
    /* 1. determine average spacing. iterate through ine two people at a time, 
    compute avg distance between pelvis joints and save average */
    averageDistance = avgDistance(body_frame);
    /* 2. determine the last person in the line */
    lastPerson = getLastPerson(body_frame);
    /* 3. create the fitted line */
    fittedLine = getFittedLine(body_frame);
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
void MKPComputeGoal::polyfit(const std::vector<double> &t, const std::vector<double> &v, std::vector<double> &coeff, int order)
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
	coeff.resize(order+1);
	for (int k = 0; k < order+1; k++)
	{
		coeff[k] = result[k];
	}

}
 

std::vector<float> MKPComputeGoal::getFittedLine(k4abt::frame bodyFrame) {
    std::vector<double> xpos;
    std::vector<double> zpos;
    std::vector<double> coeff;
    std::vector<float> fitted_line;

    // get fitted line
    for (int i = 0; i < bodyFrame.get_num_bodies(); i++) {
        k4abt_skeleton_t skeleton = bodyFrame.get_body_skeleton(i);
        float* position = calculateJointPos(skeleton, K4ABT_JOINT_PELVIS);
        xpos.push_back(position[0]);
        zpos.push_back(position[2]);
    }

    // get fitted function
    polyfit(xpos, zpos, coeff, 1);
    for (int i = 0; i < xpos.size(); i++) {
        double zfitted = coeff[0] + coeff[1]*xpos.at(i) + coeff[2]*(pow(xpos.at(i), 2)) +coeff[3]*(pow(xpos.at(i), 3));
        fitted_line.push_back(zfitted);
    }

    return fitted_line;
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