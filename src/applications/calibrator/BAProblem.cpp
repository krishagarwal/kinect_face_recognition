#include "BAProblem.h"
#include "ReprojectionCostFunctor.h"
#include "assert.h"
#include <ceres/ceres.h>
#include <iostream>
#include <vision_geometry/HomCartShortcuts.h>

using std::cout;
using std::endl;
using std::vector;
using namespace Eigen;
using namespace ceres;

/**
 * @brief Construct a new BAProblem::BAProblem object
 *
 * @param nCameras Number of cameras in scene
 * @param k Camera intrinsic matrix
 * @param samples Vector of CalibrationInput - each containing info from 1 target in 1 time slice
 */
BAProblem::BAProblem(int nCameras, const MatrixXd& k, const k4a::calibration calib, const std::vector<CalibrationInput>& samples)
    : _calib(calib)
    , _samples(samples)
{
    // Give identity to all RTs
    for (int i = 0; i < nCameras; i++) {
        _camRTs.push_back(vector<double> { 0, 0, 0, 1, 0, 0, 0 });
    }
    // Need to compute RT for each "sample" (target in point in time)
    for (int i = 0; i < samples.size(); i++) {
        // _targetRTs.push_back(vector<double> { 0, 0, 0, 1, 0, 0, 0 });
        _targetRTs.push_back(BAProblem::rTMatToVec(samples[i].estimatedPose));
    }

    auto p = calib.color_camera_calibration.intrinsics.parameters.param;
    _intrinsics << p.fx, 0, p.cx,
        0, p.fy, p.cy,
        0, 0, 1;
}

/**
 * @brief Add estimates for camera positions
 *
 * @param camRTs
 */
void BAProblem::setCamEstimation(const std::vector<MatrixXd>& camRTs)
{
    // Check there is RT for every camera
    assert(camRTs.size() == _camRTs.size());

    // Directly set these
    for (int i = 0; i < camRTs.size(); i++) {
        _camRTs[i] = BAProblem::rTMatToVec(camRTs[i]);
    }

    // TODO figure out best way to set estimates for targets (currently depends on being set in CalibrationInput)
}

void BAProblem::setTargetEstimation(const std::vector<MatrixXd>& targetRTs)
{
    // // Check there is RT for every camera
    // assert(camRTs.size() == _camRTs.size());

    // // Directly set these
    // for (int i = 0; i < camRTs.size(); i++) {
    //     _camRTs[i] = BAProblem::rTMatToVec(camRTs[i]);
    // }

    // // TODO figure out best way to set estimates for targets
}

/**
 * @brief Solve currently set bundle adjustment problem
 *
 */
void BAProblem::solve()
{
    Problem problem;

    vector<ResidualBlockId> to_eval;

    // Set up residual blocks
    for (int sampleId = 0; sampleId < _samples.size(); sampleId++) {
        const CalibrationInput& sample = _samples[sampleId];
        // Add a residual every time a camera can see points
        for (auto const& pair : sample.imagePoints) {
            int camId = pair.first;
            Eigen::MatrixXd modelPoints = sample.modelPoints;
            Eigen::MatrixXd imagePoints = sample.imagePoints.at(camId);

            for (int i = 0; i < imagePoints.cols(); i++) {
                // Build blocks
                // TODO add enum for switching between these?
                ReprojectionCostFunctorKinectProjection* rcf = new ReprojectionCostFunctorKinectProjection(_calib, modelPoints.col(i), imagePoints.block<2, 1>(0, i));
                CostFunction* costFunction = new AutoDiffCostFunction<ReprojectionCostFunctorKinectProjection, 2, 7, 7>(rcf);
                // ReprojectionCostFunctor* rcf = new ReprojectionCostFunctor(_intrinsics, modelPoints.col(i), imagePoints.block<2, 1>(0, i));
                // CostFunction* costFunction = new AutoDiffCostFunction<ReprojectionCostFunctor, 2, 7, 7>(rcf);

                ResidualBlockId r_id = problem.AddResidualBlock(costFunction, nullptr, _camRTs.at(camId).data(), _targetRTs.at(sampleId).data());

                to_eval.push_back(r_id);
            }
        }
    }

    vector<double*> params;
    problem.GetParameterBlocks(&params);
    // Set local parameterizations
    for (double* param : params) {
        ProductParameterization* se3_param = new ProductParameterization(
            new EigenQuaternionParameterization(), new IdentityParameterization(3));
        problem.SetParameterization(param, se3_param);
    }
    // Set cam 0 to be fixed
    problem.SetParameterBlockConstant(_camRTs.at(0).data());

    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = DENSE_NORMAL_CHOLESKY;
    // options.function_tolerance = 1e-16;
    options.parameter_tolerance = 1e-18;
    options.gradient_tolerance = 1e-18;
    options.function_tolerance = 1e-18;
    options.max_num_iterations = 1000;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 12;

    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;

    // Evaluation to get individual/total residuals
    ceres::Problem::EvaluateOptions options2;
    options2.residual_blocks = to_eval;
    double total_cost = 0.0;
    vector<double> evaluated_residuals;
    problem.Evaluate(options2, &total_cost, &evaluated_residuals, nullptr, nullptr);
    // for (auto i = 0; i < evaluated_residuals.size(); i++) {
    //     cout << i << ": " << evaluated_residuals[i] << endl;
    // }
    cout << "Total cost: " << total_cost << endl;
}

/**
 * @brief Returns current camera transforms for BAProblem
 *
 * @return vector<MatrixXd>
 */
vector<MatrixXd> BAProblem::getCameraRTs()
{
    vector<MatrixXd> res;
    for (auto const& x : _camRTs) {
        res.push_back(BAProblem::rTvecToMat(x));
    }
    return res;
}

// Draw 2D coordinates
void drawPoints2(Eigen::MatrixXd& mat, cv::Mat& img, cv::Scalar color)
{
    for (int i = 0; i < mat.cols(); i++) {
        cv::circle(img, cv::Point(mat(0, i), mat(1, i)), 3, color, 10);
    }
}

/**
 * @brief Project model points into a camera frame
 *
 * @param camId Desired camera frame
 * @param sampleId Desired sample
 * @param modelPoints 3D homogenous coordinates of model points
 * @return MatrixXd
 */
MatrixXd BAProblem::projectPoints(int camId, int sampleId, const MatrixXd& modelPoints)
{
    // Transform model points (AprilTag frame) into world coordinate frame
    Eigen::MatrixXd worldPoints = BAProblem::rTvecToMat(_targetRTs[sampleId]).inverse() * modelPoints;
    Eigen::MatrixXd observedPoints = _intrinsics * BAProblem::rTvecToMat(_camRTs[camId]).block<3, 4>(0, 0) * worldPoints;
    return divideByLastRowRemoveLastRow(observedPoints);
}

/**
 * @brief Compute reprojected error manually based on current transforms
 *
 * @param show Visualize with cv::imshow
 */
void BAProblem::checkReprojectionError(bool show)
{
    double error = 0;
    double residual = 0; // apply squaring
    // Go through all CalibrationInputs and compute reprojection error using
    // current transforms
    for (int sampleId = 0; sampleId < _samples.size(); sampleId++) {
        const CalibrationInput& sample = _samples[sampleId];
        // Check each camID
        for (auto const& pair : sample.imagePoints) {
            int camId = pair.first;
            Eigen::MatrixXd modelPoints = sample.modelPoints;
            Eigen::MatrixXd imagePoints = sample.imagePoints.at(camId);

            // Project points into cam and compute reprojection error
            MatrixXd observedPoints = projectPoints(camId, sampleId, modelPoints);
            imagePoints = divideByLastRowRemoveLastRow(imagePoints);
            for (int r = 0; r < observedPoints.rows(); r++) {
                for (int c = 0; c < observedPoints.cols(); c++) {
                    double diff = observedPoints(r, c) - imagePoints(r, c);
                    error += diff < 0 ? -1 * diff : diff; // changed
                    residual += diff * diff;
                }
            }

            if (show) {
                // Drawing for visualization
                cv::Mat img = sample.images.at(camId);
                drawPoints2(observedPoints, img, cv::Scalar(0, 0, 255));
                drawPoints2(imagePoints, img, cv::Scalar(0, 255, 0));

                cout << observedPoints << endl;
                cout << imagePoints << endl;
                cout << endl;

                cv::resize(img, img, cv::Size(), 0.25, 0.25);
                cv::imshow("Red = points projected, Green = points detected", img);
                cv::waitKey(0);
            }
        }
    }
    cout << "reprojection error: " << error << endl;
    cout << "residual: " << residual << endl;
}

/**
 * @brief Convert vector (x,y,z,w)(x,y,z) to Eigen matrix
 *
 * @return Eigen::MatrixXd 4x4 rigid transformation
 */
MatrixXd BAProblem::rTvecToMat(const std::vector<double>& v)
{
    assert(v.size() == 7);
    Quaterniond q(v[3], v[0], v[1], v[2]);
    Matrix3d rotMat = q.toRotationMatrix();
    MatrixXd res = Matrix4d::Identity();
    res.block<3, 3>(0, 0) = rotMat;
    res(0, 3) = v[4];
    res(1, 3) = v[5];
    res(2, 3) = v[6];
    return res;
}

/**
 * @brief Convert vector Eigen::Matrix4d to 7-dim vector
 *
 * @return 7-dim vector, (x,y,z,w)(x,y,z)
 */
std::vector<double> BAProblem::rTMatToVec(const MatrixXd& mat)
{
    assert(mat.rows() == 4 && mat.cols() == 4);
    assert(mat(3, 3) == 1);

    Matrix3d rotMat(mat.block(0, 0, 3, 3));
    Quaterniond q(rotMat);
    return vector<double> { q.x(), q.y(), q.z(), q.w(), mat(0, 3), mat(1, 3), mat(2, 3) };
}