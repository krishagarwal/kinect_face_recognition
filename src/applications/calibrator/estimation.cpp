#include "vision_geometry/CameraIntrinsics.h"

#include "KinectWrapper.h"
#include "vision_geometry/CVUtil.h"
#include "vision_geometry/HomographyShortcuts.h"
#include "vision_geometry/HomCartShortcuts.h"
#include "vision_geometry/LinearAlgebraShortcuts.h"

#include "estimation.h"
#include <iostream>

using namespace Eigen;

// projectedPoints represents the 2D points to draw. It has two or three rows; the X is row 0 / row 2, the Y is row 1 / row 2.
// Each column is one point.
void drawPoints(cv::Mat background, Eigen::MatrixXd projectedPoints) {
    cv::Mat image;
    background.copyTo(image);

    // std::vector<cv::Point2d> cvPoints;
    for (int point = 0; point < projectedPoints.cols(); point++) {
        double x = projectedPoints(0, point);
        double y = projectedPoints(1, point);
        if (projectedPoints.rows() > 2) {
            x /= projectedPoints(2, point);
            y /= projectedPoints(2, point);
        }
        //cvPoints.push_back(cv::Point2d(x, y));
        if (point == 0) drawCrosshair(image, x, y, 10, 2, cv::Scalar(0, 0, 255));
        else if (point == 1) drawCrosshair(image, x, y, 10, 2, cv::Scalar(0, 255, 0));
        else drawCrosshair(image, x, y, 10, 2, cv::Scalar(255, 0, 0));
    }
    //cv::drawChessboardCorners(image, cv::Size2d(chessboardCols, chessboardRows), cvPoints, true);

    cv::Mat scaled;
    cv::resize(image, scaled, cv::Size(), 0.4, 0.4);
    cv::imshow("camera", scaled);
    cv::waitKey();
}

void printMatrix(MatrixXd mat);

MatrixXd homographyToRT(MatrixXd homography) {
    Vector3d r1 = homography.col(0), r2 = homography.col(1);
    double r1Norm = r1.norm(), r2Norm = r2.norm();
    r1.normalize();
    r2.normalize();
    Vector3d r3 = r1.cross(r2);

    Vector3d t = homography.col(2);
    t /= 0.5 * (r1Norm + r2Norm);    

    Matrix3d rot(3, 3);
    rot.col(0) = r1;
    rot.col(1) = r2;
    rot.col(2) = r3;
    
    MatrixXd transform = MatrixXd::Identity(4, 4);
    transform.block(0, 0, 3, 3) = rot;
    transform.block(0, 3, 3, 1) = t;

    return transform;
}

void printMatrix(MatrixXd mat) {
    for (size_t row = 0; row < mat.rows(); row++) {
        for (size_t col = 0; col < mat.cols(); col++) {
            std::cout << mat(row, col);
            if (col == mat.cols() - 1) {
                if (row == mat.rows() - 1) {
                    std::cout << ";\n";
                } else {
                    std::cout << ",\n";
                }
            } else {
                std::cout << ", ";
            }
        }
    } 
}

MatrixXd createIsometricConversionHomography(MatrixXd pts) {
    float centroidX = 0;
    float centroidY = 0;
    for(int i = 0; i < pts.cols(); i++) {
        centroidX += pts(0, i);
        centroidY += pts(1, i);
    }

    centroidX /= pts.cols();
    centroidY /= pts.cols();

    float scale = 0;
    for(int i = 0; i < pts.cols(); i++) {
        float scaleX = pts(0, i) - centroidX;
        float scaleY = pts(1, i) - centroidY;
        scale += sqrt(scaleX * scaleX + scaleY * scaleY);
    }

    scale /= pts.cols();

    scale = sqrt(2.0) / scale;

    Eigen::MatrixXd isoMat(3,3);
    isoMat(0,0) = scale;
    isoMat(0,1) = 0.0;
    isoMat(0,2) = -scale * centroidX;

    isoMat(1,0) = 0.0;
    isoMat(1,1) = scale;
    isoMat(1,2) = -scale * centroidY;

    isoMat(2,0) = 0.0;
    isoMat(2,1) = 0.0;
    isoMat(2,2) = 1.0;

    return isoMat;
}

MatrixXd estimateteRTModelToCamera(MatrixXd modelPointsH2D, MatrixXd camPointsH2D, MatrixXd intrinsicHomography) {
    Eigen::MatrixXd isoMat = createIsometricConversionHomography(camPointsH2D);

    Eigen::MatrixXd normalizedHomography = computeDLTHomography(modelPointsH2D, isoMat * camPointsH2D);
    
    MatrixXd H = isoMat.inverse() * normalizedHomography;
    MatrixXd chessboardToCameraHomography = intrinsicHomography.inverse() * H;
    return homographyToRT(chessboardToCameraHomography);
}

std::string printMathematicaString(Eigen::MatrixXd A) {
    std::stringstream ss;
    ss << std::fixed;
    ss.precision(8);
    ss << "{";
    for(size_t row = 0; row < A.rows(); row++) {
        ss << "{";
        for(size_t col = 0; col < A.cols(); col++) {
            ss << A(row, col);
            if(col < (A.cols() - 1))
                ss << ",";
        }
        ss << "}";
        if(row < (A.rows() - 1))
            ss << ",";
    }
    ss << "}";
    return ss.str();
}
