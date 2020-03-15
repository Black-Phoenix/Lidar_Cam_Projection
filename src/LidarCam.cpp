//
// Created by raven on 3/12/20.
//

#include "../include/lidar_proj/LidarCam.h"

using namespace lidar_proj;

void lidar_proj::LidarCam::init() {
    // Init image_transport
    it_ = new image_transport::ImageTransport(nh_);
    nh_.param<int>("img_W", params_.img_W, 1024);
    nh_.param<int>("img_H", params_.img_H, 720);
    nh_.param<int>("lidar_W", params_.lidar_W, 1024);
    nh_.param<int>("lidar_H", params_.lidar_H, 64);
    nh_.param<int>("num_cams", params_.num_cams, 1);
    assert(params_.num_cams <= 4); // Current max we support
    nh_.param<string>("calib_file", params_.config_path,
                      "/home/raven/Code/ROS_ws/stereocam_ws/src/lidar_proj/calib/calib");
    // Create camera and lidar objects
    for (int cam = 0; cam < params_.num_cams; cam++) {
        params_.init_angle.push_back(0.0);
        nh_.param<float>("init_angle" + to_string(cam), params_.init_angle[cam], 0.0);
    }
    lidars_.push_back(new Lidar(params_));
    // Create publishers and subscribers
    painted_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("painted_pc", 1);
    lidar_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "pointcloud", 1);
    for (int cam = 0; cam < params_.num_cams; cam++) {
        sparse_depth_pub_.emplace_back(nh_.advertise<sensor_msgs::Image>("depth_img_" + to_string(cam), 1));
        img_subs_.push_back(new image_transport::SubscriberFilter(*it_, "rgb" + to_string(cam), 10)); // todo fix this
    }
    // Create sync policies
    //This is kind of an annoying if statement series, but due to typing, have to do it this way. Thanks Ian
    if (params_.num_cams == 1) {
        sync_single_ = new message_filters::Synchronizer<policy_single>(policy_single(10), *lidar_sub_, *img_subs_[0]);
        sync_single_->registerCallback(boost::bind(&LidarCam::cam_lidar_callback, this, _1, _2,
                                                   sensor_msgs::Image::ConstPtr(),
                                                   sensor_msgs::Image::ConstPtr(),
                                                   sensor_msgs::Image::ConstPtr()));
    } else if (params_.num_cams == 2) {
        sync_dual_ = new message_filters::Synchronizer<policy_dual>(policy_dual(10), *lidar_sub_,
                                                                    *img_subs_[0],
                                                                    *img_subs_[1]);
        sync_dual_->registerCallback(boost::bind(&LidarCam::cam_lidar_callback, this, _1, _2, _3,
                                                 sensor_msgs::Image::ConstPtr(),
                                                 sensor_msgs::Image::ConstPtr()));
    } else if (params_.num_cams == 3) {
        sync_triple_ = new message_filters::Synchronizer<policy_triple>(policy_triple(10), *lidar_sub_,
                                                                        *img_subs_[0],
                                                                        *img_subs_[1],
                                                                        *img_subs_[2]);
        sync_triple_->registerCallback(boost::bind(&LidarCam::cam_lidar_callback, this, _1, _2, _3, _4,
                                                   sensor_msgs::Image::ConstPtr()));
    } else if (params_.num_cams == 4) {
        sync_quad_ = new message_filters::Synchronizer<policy_quad>(policy_quad(10), *lidar_sub_,
                                                                    *img_subs_[0],
                                                                    *img_subs_[1],
                                                                    *img_subs_[2],
                                                                    *img_subs_[3]);
        sync_quad_->registerCallback(boost::bind(&LidarCam::cam_lidar_callback, this, _1, _2, _3, _4, _5));
    }
}

void lidar_proj::LidarCam::cam_lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                                              const sensor_msgs::Image::ConstPtr &img0_msg,
                                              const sensor_msgs::Image::ConstPtr &img1_msg,
                                              const sensor_msgs::Image::ConstPtr &img2_msg,
                                              const sensor_msgs::Image::ConstPtr &img3_msg) {
    ROS_INFO("PC callback active");
    vector<cv_bridge::CvImageConstPtr> imgs{cv_bridge::toCvCopy(img0_msg), cv_bridge::toCvCopy(img1_msg),
                                                 cv_bridge::toCvCopy(img2_msg), cv_bridge::toCvCopy(img3_msg)};
    pcl::PointCloud<PointOS1> cloud{};
    pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud{};
    pcl::fromROSMsg(*cloud_msg, cloud);
    rgb_cloud.points.resize(cloud.size());
    rgb_cloud.height = cloud.height;
    rgb_cloud.width = cloud.width;
    int W = lidars_[0]->W_;
    int H = lidars_[0]->H_;
    std::vector<int> px_offset = lidars_[0]->px_offset_;
    // Fix distortion by lidar beam
    for (int v = 0; v < W; v++) {
        for (int u = H - 1; u >= 0; u--) {
            int vv = (v + px_offset[u]) % W;

            int index = vv * H + u;

            int vv_f = v % W;
            int index_f = vv_f * H + u;

            rgb_cloud.points[index_f].x = cloud.points[index].x;
            rgb_cloud.points[index_f].y = cloud.points[index].y;
            rgb_cloud.points[index_f].z = cloud.points[index].z;
            //Pack rgb value
            uint8_t r = 255, g = 255, b = 255;
            uint32_t rgb = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
            rgb_cloud.points[index_f].rgb = *reinterpret_cast<float *>(&rgb);
        }
    }
    // Now we create the colored pc and image
    project_points(rgb_cloud, imgs);
    // Publish results

    // painted pc
    // sparce depth imgs
}

void lidar_proj::LidarCam::project_points(pcl::PointCloud<pcl::PointXYZRGB> &rgb_cloud,
                                          const vector<cv_bridge::CvImageConstPtr> &rgb_imgs) {
// For each camera, find the projection
    for (int cam = 0; cam < params_.num_cams; cam++) {
        lidars_[0]->virtual_cams_[cam]->reset_img();
        for (int v = 0; v < lidars_[0]->W_; v++) {
            float diff = std::abs(v * 2 * M_PI / lidars_[0]->W_ - lidars_[0]->virtual_cams_[cam]->init_angle_);
            while (diff > M_PI) diff -= 2 * M_PI;
            if (std::abs(diff) > M_PI / 4) continue;

            //projectPoints takes a vector of points, even if the vector only has one entry.
            vector<cv::Point2f> pts_img;
            vector<cv::Point3f> obj_pts;
            vector<int> indices;
            //homogeneous coordinates
            cv::Mat pt_h(4, 1, CV_32FC1);

            for (int u = lidars_[0]->H_ - 1; u >= 0; u--) {
                //Find the real u,v coordinates in the point cloud
                int vv = v % lidars_[0]->W_;
                int index = vv * lidars_[0]->H_ + u;
                const auto &pt3 = rgb_cloud[index];

                pt_h.at<float>(0) = pt3.y;
                pt_h.at<float>(1) = -pt3.z;
                pt_h.at<float>(2) = -pt3.x;
                pt_h.at<float>(3) = 1;

                //Transform to camera frame
                cv::Mat pt_h_trans = (lidars_[0]->virtual_cams_[cam]->inv_T_ * pt_h)(cv::Rect(0, 0, 1, 3));

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
                                  cv::Mat::zeros(3, 1, CV_32FC1), lidars_[0]->virtual_cams_[cam]->K_,
                                  lidars_[0]->virtual_cams_[cam]->D_, pts_img);
                int img_map_y = lidars_[0]->W_;
                int idx = 0;
                for (auto pt_img : pts_img) {
                    int index = indices[idx++];

                    if ((int) pt_img.x > 0 && (int) pt_img.x < rgb_imgs[cam]->image.cols &&
                        (int) pt_img.y > 0 && (int) pt_img.y < rgb_imgs[cam]->image.rows) {
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
                            cv::Vec3b rgb_px =  rgb_imgs[cam]->image.at<cv::Vec3b>((int) pt_img.y, (int) pt_img.x);
                            //Opencv is BGR because why follow standards?
                            uint32_t rgb = ((uint32_t) rgb_px[2] << 16 | (uint32_t) rgb_px[1] << 8 |
                                            (uint32_t) rgb_px[0]);
                            rgb_cloud.points[index].rgb = *reinterpret_cast<float *>(&rgb);
                        }
                    }
                }
            }
        }
    }
}

lidar_proj::LidarCam::~LidarCam() {
//    for (int i = 0; i < cams_.size(); i++)
//        delete cams_[i];
    for (int i = 0; i < lidars_.size(); i++)
        delete lidars_[i];
    for (int i = 0; i < img_subs_.size(); i++)
        delete img_subs_[i];
    delete it_;
    delete lidar_sub_;
    delete sync_single_;
    delete sync_dual_;
    delete sync_triple_;
    delete sync_quad_;
}
