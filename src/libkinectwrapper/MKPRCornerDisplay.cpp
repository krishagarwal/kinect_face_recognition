#include "include/MKPRCornerDisplay.h"
#include "include/MultiKinectPacket.h"
#include "include/MKConfig.h"
#include <opencv2/core.hpp>

using namespace cv;

void MKPRCornerDisplay::receiveFrame(MultiKinectPacket &mkp) {
    for(size_t i = 0; i < mkp.getConfig()._nCameras; i++) {
        // TODO should recipients or wrapper call conversion?
        // mkp.convertKinectImageToBGRA();
        // mkp.convertBGRAToRGB();
        cv::Mat img = mkp[i].getRGBColor();
        if(mkp[i].cornerDetections.count("apriltag")){
            Eigen::MatrixXd detections = mkp[i].cornerDetections["apriltag"];
            for(int j = 0; j < detections.cols(); j++){
                cv::Point point(detections(0,j), detections(1,j));
                cv::circle(img, point, 5, cv::Scalar(255, 0, 255), -3);

                // Draw scaled version for rgb960x540
                cv::Point scaledPoint(point.x / 4, point.y / 4);
                cv::circle(mkp[i].getRGBColorPreviewScale(), scaledPoint, 5, cv::Scalar(255, 0, 255), -3);
            }
        }
    }
}
