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
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

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
    void project_points(pcl::PointCloud<pcl::PointXYZRGB> &rgb_cloud, const sensor_msgs::Image::ConstPtr& img_msg) {
// For each camera, find the projection
        for (int cam = 0; cam < virtual_cams_.size(); cam++) {
            virtual_cams_[cam]->reset_img();
            cv_bridge::CvImagePtr rgb_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            for (int v = 0; v < W_; v++) {
                float diff = std::abs(v * 2 * M_PI / W_ - virtual_cams_[cam]->init_angle_);
                diff = atan2(sin(diff), cos(diff));
//                if (std::abs(diff) > M_PI / 2) continue;

                //projectPoints takes a vector of points, even if the vector only has one entry.
                vector<cv::Point2f> pts_img;
                vector<cv::Point3f> obj_pts;
                vector<int> indices;
                //homogeneous coordinates
                cv::Mat pt_h(4, 1, CV_32FC1);

                for (int u = H_ - 1; u >= 0; u--) {
                    //Find the real u,v coordinates in the point cloud
                    int vv = v % W_;
                    int index = vv * H_ + u;
                    const auto &pt3 = rgb_cloud[index];

                    pt_h.at<float>(0) = pt3.y;
                    pt_h.at<float>(1) = -pt3.z;
                    pt_h.at<float>(2) = -pt3.x;
                    pt_h.at<float>(3) = 1;

                    //Transform to camera frame
                    cv::Mat pt_h_trans = (virtual_cams_[cam]->inv_T_ * pt_h)(cv::Rect(0, 0, 1, 3));

                    //ROS_INFO_STREAM(pt_h_trans);
                    if (pt_h_trans.at<float>(2) < 0.5 ||
                        abs(pt_h_trans.at<float>(0)) > abs(pt_h_trans.at<float>(2)))
                        continue;

                    obj_pts.push_back(
                            cv::Point3f(pt_h_trans.at<float>(0), pt_h_trans.at<float>(1), pt_h_trans.at<float>(2)));
                    indices.push_back(index);
                }

                if (obj_pts.size() > 0) {
                    cv::projectPoints(obj_pts, cv::Mat::zeros(3, 1, CV_32FC1),
                                      cv::Mat::zeros(3, 1, CV_32FC1), virtual_cams_[cam]->K_,
                                      cv::Mat::zeros(4,1,CV_32FC1), pts_img);
//                    cv::projectPoints(obj_pts, cv::Mat::zeros(3, 1, CV_32FC1),
//                                      cv::Mat::zeros(3, 1, CV_32FC1), virtual_cams_[cam]->K_,
//                                      virtual_cams_[cam]->D_, pts_img);
                    int img_map_y = W_;
                    int idx = 0;
                    for (auto pt_img : pts_img) {
                        int index = indices[idx++];

                        if ((int) pt_img.x > 0 && (int) pt_img.x < rgb_img->image.cols &&
                            (int) pt_img.y > 0 && (int) pt_img.y < rgb_img->image.rows) {
                            bool ok = true;
                            if (pt_img.y > img_map_y) {
                                rgb_cloud[index].x = 0;
                                rgb_cloud[index].y = 0;
                                rgb_cloud[index].z = 0;
                                ok = false;
                            } else {
                                img_map_y = pt_img.y;
                            }

                            if (ok) {
                                cv::Vec3b rgb_px =  rgb_img->image.at<cv::Vec3b>((int) pt_img.y, (int) pt_img.x);
                                //Opencv is BGR because why follow standards?
                                uint32_t rgb = ((uint32_t) rgb_px[2] << 16 | (uint32_t) rgb_px[1] << 8 |
                                                (uint32_t) rgb_px[0]);
                                rgb_cloud.points[index].rgb = *reinterpret_cast<float *>(&rgb);
                                cv::Vec3f & pos = virtual_cams_[cam]->img_.at<cv::Vec3f>((int)pt_img.y,(int)pt_img.x);
                                pos[0] = rgb_cloud.points[index].x;
                                pos[1] = rgb_cloud.points[index].y;
                                pos[2] = rgb_cloud.points[index].z;
                            }
                        }
                    }
                }
            }
        }
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
