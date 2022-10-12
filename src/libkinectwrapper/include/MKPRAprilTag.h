#ifndef MKPR_APRIL_TAG_H
#define MKPR_APRIL_TAG_H

#include "MKPRecipient.h"
#include <apriltag/apriltag.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

class MKPRAprilTag : public MKPRecipient {
public:
    MKPRAprilTag();
    void receiveFrame(MultiKinectPacket &mkp);
    
protected:
    Eigen::MatrixXd detectCorners(const cv::Mat &img);
    apriltag_detector_t *td;
    apriltag_family_t *tf;
};

#endif