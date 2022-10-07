#include <Eigen/Eigen>

Eigen::MatrixXd estimateteRTModelToCamera(Eigen::MatrixXd modelPointsH2D, Eigen::MatrixXd camPointsH2D,
    Eigen::MatrixXd intrinsicHomography);
