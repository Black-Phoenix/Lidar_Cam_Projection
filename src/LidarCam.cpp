//
// Created by raven on 3/12/20.
//

#include "../include/lidar_proj/LidarCam.h"

using namespace lidar_proj;

void lidar_proj::LidarCam::init() {
    // Init image_transport
    it_ = new image_transport::ImageTransport(nh_);
    nh_.param<int>("W", params_.W, 1024);
    nh_.param<int>("H", params_.H, 64);
    nh_.param<int>("num_cams", params_.num_cams, 1); // todo fix this
    assert(params_.num_cams < 4); // Current max we support
    nh_.param<string>("calib_file", params_.config_path,
                      "/home/raven/Code/ROS_ws/stereocam_ws/src/lidar_proj/calib/calib");
    // Create camera and lidar objects
    for (int cam = 0; cam < params_.num_cams; cam++)
        cams_.emplace_back(new Camera(params_.config_path + to_string(cam) + ".yml"));
    lidars_.push_back(new Lidar(params_.H, params_.W, params_.config_path, params_.num_cams));
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
    std::vector<sensor_msgs::Image::ConstPtr> imgs{img0_msg, img1_msg, img2_msg, img3_msg};
}