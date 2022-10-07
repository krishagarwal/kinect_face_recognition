#ifndef BA_PROBLEM_H
#define BA_PROBLEM_H

#include "CalibrationInput.h"
#include <Eigen/Dense>
#include <k4a/k4a.hpp>
#include <vector>

/**
 * @brief Wrapper for bundle adjustment problem of multiple cameras and AprilTag targets
 *        
 */
class BAProblem {
public:
    BAProblem(int nCameras, const Eigen::MatrixXd& k, const k4a::calibration calib, const std::vector<CalibrationInput>& samples);
    void setCamEstimation(const std::vector<Eigen::MatrixXd>& camRTs);
    void setTargetEstimation(const std::vector<Eigen::MatrixXd>& targetRTs);
    void solve();
    std::vector<Eigen::MatrixXd> getCameraRTs();
    void checkReprojectionError(bool);

    // utils
    static Eigen::MatrixXd rTvecToMat(const std::vector<double>&);
    static std::vector<double> rTMatToVec(const Eigen::MatrixXd&);

protected:
    Eigen::MatrixXd projectPoints(int camId, int sampleId, const Eigen::MatrixXd& modelPoints);


    Eigen::Matrix3d _intrinsics;
    const k4a::calibration _calib;
    std::vector<CalibrationInput> _samples;
    // Not pretty but represent rigid transforms as 7-dim vectors (x,y,z,w)(x,y,z) for sovler purposes
    std::vector<std::vector<double>> _camRTs;
    std::vector<std::vector<double>> _targetRTs;
    
};

#endif