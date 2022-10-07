#ifndef REPROJECTION_COST_FUNCTOR_H
#define REPROJECTION_COST_FUNCTOR_H

#include "assert.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <Eigen/Dense>
#include <k4a/k4a.hpp>

using namespace Eigen;
using namespace std;

/**
 * @brief Functor to use Azure Kinect SDK 3d->2d projection
 *        may perform better because of handling distortion parameters
 */
class ComputeProjectionFunctor {
public:
    ComputeProjectionFunctor(const k4a::calibration calib)
        : _calib(calib)
    {
    }
    bool operator()(const double* imagePoint2d, const double* point, double* residuals) const
    {
        k4a_float2_t projected;
        _calib.convert_3d_to_2d({ static_cast<float>(point[0]),
                                    static_cast<float>(point[1]),
                                    static_cast<float>(point[2]) },
            K4A_CALIBRATION_TYPE_COLOR,
            K4A_CALIBRATION_TYPE_COLOR,
            &projected);

        residuals[0] = imagePoint2d[0] - projected.xy.x;
        residuals[1] = imagePoint2d[1] - projected.xy.y;
        return true;
    }

protected:
    const k4a::calibration _calib;
};

/**
 * @brief Computes reprojection error, uses 7-d RT as parameter for camera and target
 *        Projects 3d point to image frame using Azure Kinect SDK
 */
class ReprojectionCostFunctorKinectProjection {
public:
    ReprojectionCostFunctorKinectProjection(const k4a::calibration calib, const Vector4d& modelPoints, const Vector2d& imagePoints)
        : _calib(calib)
    {
        compute_projection.reset(new ceres::CostFunctionToFunctor<2, 2, 3>(
            new ceres::NumericDiffCostFunction<ComputeProjectionFunctor,
                ceres::CENTRAL, 2, 2, 3>(
                new ComputeProjectionFunctor(calib))));
        model_X = modelPoints[0];
        model_Y = modelPoints[1];
        model_Z = modelPoints[2];
        model_W = modelPoints[3];

        image_X = imagePoints[0];
        image_Y = imagePoints[1];
    }

    template <typename T>
    bool operator()(const T* const camRt, const T* const objRt, T* residuals) const
    {
        // Cam pose params
        T camq[4];
        camq[0] = camRt[0];
        camq[1] = camRt[1];
        camq[2] = camRt[2];
        camq[3] = camRt[3];

        T camt[3];
        camt[0] = camRt[4];
        camt[1] = camRt[5];
        camt[2] = camRt[6];

        Quaternion<T> camQ = Map<const Eigen::Quaternion<T>>(camq);
        Matrix<T, 3, 1> camT = Map<const Eigen::Matrix<T, 3, 1>>(camt);
        Matrix<T, 3, 4> camRT;
        camRT.block(0, 0, 3, 3) = camQ.toRotationMatrix();
        camRT.block(0, 3, 3, 1) = camT;

        // Object pose params
        T objq[4];
        objq[0] = objRt[0];
        objq[1] = objRt[1];
        objq[2] = objRt[2];
        objq[3] = objRt[3];

        T objt[3];
        objt[0] = objRt[4];
        objt[1] = objRt[5];
        objt[2] = objRt[6];

        Quaternion<T> objQ = Map<const Eigen::Quaternion<T>>(objq);
        Matrix<T, 3, 1> objT = Map<const Eigen::Matrix<T, 3, 1>>(objt);
        Matrix<T, 4, 4> objRT = Matrix<T, 4, 4>::Identity();
        objRT.block(0, 0, 3, 3) = objQ.toRotationMatrix();
        objRT.block(0, 3, 3, 1) = objT;

        // Template other ponts
        Eigen::Matrix<T, 4, 1> worldPoint;
        worldPoint << T(model_X), T(model_Y), T(model_Z), T(model_W);

        // Reproject points based on parameters for camera and object transforms
        Eigen::Matrix<T, 3, 1> res = camRT * objRT.inverse() * worldPoint;

        T imagePoints[2];
        imagePoints[0] = T(image_X);
        imagePoints[1] = T(image_Y);

        T points[3];
        points[0] = res[0];
        points[1] = res[1];
        points[2] = res[2];

        (*compute_projection)(imagePoints, points, residuals);
        return true;
    }

protected:
    const k4a::calibration _calib;
    std::unique_ptr<ceres::CostFunctionToFunctor<2, 2, 3>> compute_projection;
    double model_X;
    double model_Y;
    double model_Z;
    double model_W;
    double image_X;
    double image_Y;
};

/**
 * @brief Computes reprojection error, uses 7-d RT as parameter for camera and target
 *        Projects using just intrinsic matrix
 */
class ReprojectionCostFunctor {
public:
    ReprojectionCostFunctor(const Matrix3d& intrinsics, const Vector4d& modelPoints, const Vector2d& imagePoints)
        : _intrinsics(intrinsics)
    {
        model_X = modelPoints[0];
        model_Y = modelPoints[1];
        model_Z = modelPoints[2];
        model_W = modelPoints[3];

        image_X = imagePoints[0];
        image_Y = imagePoints[1];
    }

    template <typename T>
    bool operator()(const T* const camRt, const T* const objRt, T* residuals) const
    {
        // Cam pose params
        T camq[4];
        camq[0] = camRt[0];
        camq[1] = camRt[1];
        camq[2] = camRt[2];
        camq[3] = camRt[3];

        T camt[3];
        camt[0] = camRt[4];
        camt[1] = camRt[5];
        camt[2] = camRt[6];

        Quaternion<T> camQ = Map<const Eigen::Quaternion<T>>(camq);
        Matrix<T, 3, 1> camT = Map<const Eigen::Matrix<T, 3, 1>>(camt);
        Matrix<T, 3, 4> camRT;
        camRT.block(0, 0, 3, 3) = camQ.toRotationMatrix();
        camRT.block(0, 3, 3, 1) = camT;

        // Object pose params
        T objq[4];
        objq[0] = objRt[0];
        objq[1] = objRt[1];
        objq[2] = objRt[2];
        objq[3] = objRt[3];

        T objt[3];
        objt[0] = objRt[4];
        objt[1] = objRt[5];
        objt[2] = objRt[6];

        Quaternion<T> objQ = Map<const Eigen::Quaternion<T>>(objq);
        Matrix<T, 3, 1> objT = Map<const Eigen::Matrix<T, 3, 1>>(objt);
        Matrix<T, 4, 4> objRT = Matrix<T, 4, 4>::Identity();
        objRT.block(0, 0, 3, 3) = objQ.toRotationMatrix();
        objRT.block(0, 3, 3, 1) = objT;

        // Template other ponts
        Eigen::Matrix<T, 4, 1> worldPoint;
        worldPoint << T(model_X), T(model_Y), T(model_Z), T(model_W);

        Eigen::Matrix<T, 3, 3> intrinsics;
        intrinsics << T(_intrinsics(0, 0)), T(_intrinsics(0, 1)), T(_intrinsics(0, 2)),
            T(_intrinsics(1, 0)), T(_intrinsics(1, 1)), T(_intrinsics(1, 2)),
            T(_intrinsics(2, 0)), T(_intrinsics(2, 1)), T(_intrinsics(2, 2));

        // Reproject points based on parameters for camera and object transforms
        Eigen::Matrix<T, 3, 1> res = intrinsics * camRT * objRT.inverse() * worldPoint;

        res /= (res[2] + 1e-18);

        residuals[0] = T(image_X) - res[0];
        residuals[1] = T(image_Y) - res[1];

        return true;
    }

protected:
    // Don't make this a reference unless you want to debug for the rest of your life
    const Matrix3d _intrinsics;
    double model_X;
    double model_Y;
    double model_Z;
    double model_W;
    double image_X;
    double image_Y;
};

#endif