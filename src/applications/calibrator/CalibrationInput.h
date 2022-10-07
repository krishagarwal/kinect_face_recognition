#ifndef CALIBRATION_INPUT_H
#define CALIBRATION_INPUT_H

#include <Eigen/Dense>
#include <map>
#include <opencv2/opencv.hpp>

/**
 * @brief Calibration target as captured by multiple
 *        cameras (same time)
 */
typedef struct {
    // Homogenous 3D coords
    Eigen::MatrixXd modelPoints;
    // AprilTag id (based on target)
    int tagId;
    // Originated from imageset_<imagesetID>
    int imageSetId;

    // Key based on camera id
    std::map<int, Eigen::MatrixXd> imagePoints;
    std::map<int, cv::Mat> images;

    // Used to initialize bundle adjustment problem
    Eigen::MatrixXd estimatedPose;
} CalibrationInput;

#endif