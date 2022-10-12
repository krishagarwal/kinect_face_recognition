#include <string>
#include <exception>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "estimation.h"
#include <k4a/k4a.hpp>
#include <iterator>
#include <fstream>
#include <sstream>
#include "Calibration.h"
#include "CalibrationInput.h"
#include "BAProblem.h"

using namespace std;
using namespace cv;
using namespace Eigen;
namespace po = boost::program_options;

// Lots of test code to evaluate if apriltag pose detection is better than our own
Eigen::MatrixXd matd_to_eigen(matd_t* mat)
{
    Eigen::MatrixXd out(mat->nrows, mat->ncols);
    for (int r = 0; r < mat->nrows; r++) {
        for (int c = 0; c < mat->ncols; c++) {
            out(r, c) = MATD_EL(mat, r, c);
        }
    }
    return out;
}

Eigen::MatrixXd marker_to_cam(apriltag_pose_t pose)
{
    Eigen::MatrixXd marker_to_color_cam = Eigen::MatrixXd::Identity(4, 4);
    marker_to_color_cam.block(0, 0, 3, 3) = matd_to_eigen(pose.R);
    marker_to_color_cam.block(0, 3, 3, 1) = matd_to_eigen(pose.t);
    return marker_to_color_cam;
}

Eigen::MatrixXd checkPose(apriltag_detection_t* det, k4a::calibration calib, double tagSize)
{
    apriltag_detection_info_t info = {
        .det = det,
        .tagsize = tagSize,
        .fx = calib.color_camera_calibration.intrinsics.parameters.param.fx,
        .fy = calib.color_camera_calibration.intrinsics.parameters.param.fy,
        .cx = calib.color_camera_calibration.intrinsics.parameters.param.cx,
        .cy = calib.color_camera_calibration.intrinsics.parameters.param.cy
    };

    apriltag_pose_t pose;
    double err = estimate_tag_pose(&info, &pose);

    const double* tr = &pose.R->data[0];
    const double* tt = &pose.t->data[0];

    // cout <<"Object-space error = " << err << endl;
    // cout <<"R = [ " << tr[0] << ", " << tr[1] << ", " << tr[2] << endl;
    // cout <<"      " << tr[3] << ", " << tr[4] << ", " << tr[5] << endl;
    // cout <<"      " << tr[6] << ", " << tr[7] << ", " << tr[8] << " ]" << endl;
    // cout <<"t = [ " << tt[0] << ", " << tt[1] << ", " << tt[2] << " ]" << endl;

    return marker_to_cam(pose).inverse();
}

vector<CalibrationInput> load_dataset(string input_path, double targetSize, k4a::calibration calib) {
    vector<CalibrationInput> result;

    if (access(input_path.c_str(), F_OK) == -1) {
        cerr << "The input path does not exist";
        abort();
    }

    // dim 0: image sets
    // dim 1: images from image sets (all different cameras)
    vector<vector<Mat>> frames;

    while (true) {
        std::string frame_dir = input_path + "/imageset_" + to_string(frames.size());
        if (access(frame_dir.c_str(), F_OK) == -1) {
            break;
        }

        vector<Mat> images;
        while (true) {
            std::string image_file = frame_dir + "/cam" + to_string(images.size()) + "_bgra.jpg";
            if (access(image_file.c_str(), F_OK) == -1) {
                break;
            }
            images.push_back(imread(image_file));
        }
        frames.push_back(images);
    }

    apriltag_detector_t *detector = apriltag_detector_create();
    apriltag_family_t *family_36h11 = tag36h11_create();
    apriltag_detector_add_family(detector, family_36h11);

    for (int frame = 0; frame < frames.size(); frame++) {
        map<int, shared_ptr<CalibrationInput>> thisFrameTags;

        for (int cam = 0; cam < frames[frame].size(); cam++) {
            Mat image = frames[frame][cam];
            Mat grayscale;
            cvtColor(image, grayscale, cv::COLOR_RGB2GRAY);

            image_u8_t img = {
                .width = grayscale.cols,
                .height = grayscale.rows,
                .stride = grayscale.cols,
                .buf = grayscale.data
            };
            zarray_t *detections = apriltag_detector_detect(detector, &img);

            cout << "frame " << frame << " image (camera) " << cam << " has " << zarray_size(detections) << " detections" << endl;

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                shared_ptr<CalibrationInput> ptr;
                if (!thisFrameTags.count(det->id)) {
                    ptr = thisFrameTags[det->id] = make_shared<CalibrationInput>();
                    ptr->modelPoints = Eigen::MatrixXd(4,4);
                    double half = targetSize / 2;
                    // ptr->modelPoints <<
                    //     0, 0, targetSize, targetSize,
                    //     0, targetSize, targetSize, 0,
                    //     0, 0, 0, 0,
                    //     1, 1, 1, 1;
                    ptr->modelPoints << -half, half, half, -half,
                        half, half, -half, -half,
                        0, 0, 0, 0,
                        1, 1, 1, 1;
                } else {
                    ptr = thisFrameTags[det->id];
                }

                MatrixXd imagePoints(3,4);
                for (int p = 0; p < 4; p++) {
                    for (int c = 0; c < 2; c++) {
                        imagePoints(c, p) = det->p[p][c];
                    }
                    imagePoints(2, p) = 1;
                }
                ptr->tagId = det->id;
                ptr->imageSetId = frame;
                ptr->imagePoints[cam] = imagePoints;
                ptr->images[cam] = image;
                if(cam == 0){
                    // TODO need to set estimatedPose even if camera 0 can't see the target
                    ptr->estimatedPose = checkPose(det, calib, targetSize);
                }
            }

            apriltag_detections_destroy(detections);
        }

        for (map<int, shared_ptr<CalibrationInput>>::iterator it = thisFrameTags.begin(); it != thisFrameTags.end(); it++) {
            result.push_back(*it->second);
        }
    }
    tag36h11_destroy(family_36h11);
    apriltag_detector_destroy(detector);
    return result;
}

/**
 * @brief Create k4a::calbriation from "raw" (JSON) blob
 * 
 * @param input_path Path to raw calibration file
 * @return k4a::calibration 
 */
k4a::calibration load_calib(string input_path){
    std::ifstream file(input_path + "/calib_blob");
    std::ostringstream ss;
    ss << file.rdbuf();
    const std::string& s = ss.str();
    std::vector<char> vec(s.begin(), s.end());
    return k4a::calibration::get_from_raw(vec.data(), vec.size(), K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_2160P);
}

// Draw 2D coordinates
void drawPoints(Eigen::MatrixXd& mat, cv::Mat& img, cv::Scalar color){
    for (int i = 0; i < mat.cols(); i++) {
        cv::circle(img, cv::Point(mat(0, i), mat(1, i)), 3, color, 10);
    }
}

int main(int argc, char **argv) {
    po::options_description desc("April Tag Calibrator");
    desc.add_options()
        ("input_path,i", po::value<std::string>()->required(), "Input directory from aprilTagSnapper")
        ("output_path,o", po::value<std::string>()->required(), "Output");
    
    po::variables_map vm;
    try {
        po::store(parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (const po::error &err) {
        cerr << desc << endl;
        cerr << err.what() << endl;
        return 1;
    }

    string input_path = vm["input_path"].as<string>();
    string output_path = vm["output_path"].as<string>();

    if (access(input_path.c_str(), F_OK) == -1) {
        cerr << "The input path does not exist" << endl;;
        return 1;
    }
    std::ofstream out_test(output_path);
    if(!out_test.good()){
        cerr << "The output path is not valid" << endl;
        out_test.close();
        return 1;
    }
    out_test.close();

    k4a::calibration calib = load_calib(input_path);
    Eigen::MatrixXd intrinsicHomography(3, 3);
    auto p = calib.color_camera_calibration.intrinsics.parameters.param;
    intrinsicHomography << p.fx, 0, p.cx,
        0, p.fy, p.cy,
        0, 0, 1;

    vector<CalibrationInput> frames = load_dataset(input_path, 173, calib);
    vector<Eigen::MatrixXd> transforms;
    transforms.push_back(Eigen::MatrixXd::Identity(4,4));

    for (int i = 0; i < frames.size(); i++) {
        CalibrationInput& frame = frames[i];

        Eigen::MatrixXd modelPointsH2D(3, 4);
        modelPointsH2D.block(0, 0, 2, 4) = frame.modelPoints.block(0, 0, 2, 4);
        modelPointsH2D.block(2, 0, 1, 4) = frame.modelPoints.block(3, 0, 1, 4);

        Eigen::MatrixXd rtZero = estimateteRTModelToCamera(modelPointsH2D, frame.imagePoints.at(0), intrinsicHomography);
        Eigen::MatrixXd rtOne = estimateteRTModelToCamera(modelPointsH2D, frame.imagePoints.at(1), intrinsicHomography);

        transforms.push_back((rtZero * rtOne.inverse()).inverse());
        break;
    }


    BAProblem bundleAdjustment(2, intrinsicHomography, calib, frames);
    bundleAdjustment.setCamEstimation(transforms);
    bundleAdjustment.checkReprojectionError(false);    
    bundleAdjustment.solve();
    vector<MatrixXd> camRTs = bundleAdjustment.getCameraRTs();
    
    auto R = calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation;
    auto T = calib.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation;
     
    Eigen::Matrix4d colorToDepth;
    colorToDepth << R[0],R[1],R[2],T[0],
                    R[3],R[4],R[5],T[1],
                    R[6],R[7],R[8],T[2],
                    0,0,0,1;

    // Adjust based on color to depth transformation
    for(int i = 0; i < camRTs.size(); i++){
        camRTs[i] = colorToDepth * camRTs[i] * colorToDepth.inverse();
    }
    
    for(const auto& x: camRTs){
        cout << x << endl;
    }

    cout << "Starting write" << endl;
    Calibration::writeFile(output_path, camRTs);

    return 0;
}