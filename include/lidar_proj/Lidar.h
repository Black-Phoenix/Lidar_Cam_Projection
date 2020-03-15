//
// Created by raven on 3/12/20.
//

#ifndef LIDAR_PROJ_LIDAR_H
#define LIDAR_PROJ_LIDAR_H

#include <vector>
#include <opencv2/core.hpp>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl_ros/point_cloud.h>
#include "Camera.h"

using namespace std;
struct Params {
    int lidar_W, lidar_H;
    int lidar_mode;
    int num_cams;
    string config_path;
    int img_W, img_H;
    vector<float> init_angle;
};

struct EIGEN_ALIGN16 PointOS1 {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static inline PointOS1 make(float x, float y, float z, float intensity,
                                uint32_t t, uint16_t reflectivity, uint8_t ring,
                                uint16_t noise, uint32_t range) {
        return {x, y, z, 0.0, intensity, t, reflectivity, ring, noise, range};
    }
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(PointOS1,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                          (uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)
                                          (uint16_t, noise, noise)(uint32_t, range, range))
class Lidar {
public:
    Lidar(const Params& params){
//    Lidar(int H, int W, string config, int num_cams, cv::Size img_size) : H_(H), W_(W) {
        H_ = params.lidar_H;
        W_ = params.lidar_W;
        cv::Mat lidar_to_plane = cv::Mat::zeros(4, 4, CV_32FC1);
        lidar_to_plane.at<float>(0, 2) = -1;
        lidar_to_plane.at<float>(1, 0) = 1;
        lidar_to_plane.at<float>(2, 1) = -1;
        lidar_to_plane.at<float>(3, 3) = 1;
        cv::Mat plane_to_lidar;
        cv::invert(lidar_to_plane, plane_to_lidar);
        // Fix ouster pc indexing
        setPxOffset();
        // Read T for each virtual camera
        cv::Size img_size = cv::Size(params.img_W, params.img_H);
        for (int cam = 0; cam < params.num_cams; cam++) {
            virtual_cams_.push_back(new Camera(params.config_path + to_string(cam) + ".yml", img_size));
            virtual_cams_[cam]->T_ = plane_to_lidar * virtual_cams_[cam]->T_;
            cv::invert(virtual_cams_[cam]->T_, virtual_cams_[cam]->inv_T_);
        }
    }
    ~Lidar(){
        for (int i = 0; i < virtual_cams_.size(); i++)
            delete virtual_cams_[i];
    }
    int H_, W_;
    vector<int> px_offset_;
    vector<Camera *> virtual_cams_;
private:
    void setPxOffset() {
        auto repeat = [](int n, const std::vector<int> &v) {
            std::vector<int> res{};
            for (int i = 0; i < n; i++) res.insert(res.end(), v.begin(), v.end());
            return res;
        };
        switch (W_) {
            case 512:
                px_offset_ =  repeat(16, {0, 3, 6, 9});
            case 1024:
                px_offset_ = repeat(16, {0, 6, 12, 18});
            case 2048:
                px_offset_= repeat(16, {0, 12, 24, 36});
            default:
                px_offset_ =  std::vector<int>{64, 0};
        }
    }
};


#endif //LIDAR_PROJ_LIDAR_H
