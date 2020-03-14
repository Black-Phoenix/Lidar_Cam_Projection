//
// Created by raven on 3/12/20.
//

#ifndef LIDAR_PROJ_CAMERA_H
#define LIDAR_PROJ_CAMERA_H

#include <string.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/Image.h>

using namespace std;

class Camera {
public:
    /// Constructor that loads
    /// \param config
    Camera(const string& config) {
        cv::FileStorage calib_fs(config, cv::FileStorage::READ);
        if (!calib_fs.isOpened()) {
            ROS_ERROR("Failed to find CAMERA calib file at %s", config.c_str());
            exit(1);
        } else {
            K_ = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));
            D_ = cv::Mat(5, 1, CV_64FC1, cv::Scalar(0));
            T_ = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));
            calib_fs["K"] >> K_;
            calib_fs["D"] >> D_;
            calib_fs["T"] >> T_; // Not used in this version unless it is present in the lidar_class
        }
    }

    Camera(cv::Mat K, cv::Mat dist, cv::Size img_size) : K_(K), D_(dist), img_size_(img_size) {}


    // intrinsics
    cv::Mat K_;
    cv::Mat D_; // distortion coeff
    cv::Size img_size_;
    // Extrinsics (Can be lidar-cam or cam-cam)
    cv::Mat T_; // translation
};

#endif //LIDAR_PROJ_CAMERA_H
