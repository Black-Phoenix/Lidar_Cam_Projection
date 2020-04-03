//
// Created by raven on 3/12/20.
//

#include "../include/lidar_proj/LidarCam.h"

using namespace lidar_proj;

void lidar_proj::LidarCam::init() {
    // Init image_transport
    it_ = new image_transport::ImageTransport(nh_);
    nh_.getParam("/lidar_proj/img_W", params_.img_W);
    nh_.getParam("/lidar_proj/img_H", params_.img_H);
    nh_.getParam("/lidar_proj/lidar_W", params_.lidar_W);
    nh_.getParam("/lidar_proj/lidar_H", params_.lidar_H);
    nh_.getParam("/lidar_proj/num_cams", params_.num_cams);
    assert(params_.num_cams <= 4); // Current max we support
    nh_.getParam("/lidar_proj/calib_file", params_.config_path);
    // Create camera and lidar objects
    for (int cam = 0; cam < params_.num_cams; cam++) {
        params_.init_angle.push_back(0.0);
        nh_.getParam("/lidar_proj/init_angle" + to_string(cam), params_.init_angle[cam]);
    }
    lidars_.push_back(new Lidar(params_));
    // Create publishers and subscribers
    painted_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("painted_pc", 1);
    lidar_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/os1_cloud_node/points", 1);
    for (int cam = 0; cam < params_.num_cams; cam++) {
        sparse_depth_pub_.emplace_back(nh_.advertise<sensor_msgs::Image>("depth_img_" + to_string(cam), 1));
        img_subs_.push_back(new image_transport::SubscriberFilter(*it_, "/camera/image_color/", 10)); // todo fix this
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
//    ROS_INFO("PC callback active");
//    vector<cv_bridge::CvImagePtr> imgs;
//    if (params_.num_cams >= 1)
//        imgs.push_back(cv_bridge::toCvCopy(img0_msg));
//    if (params_.num_cams >= 2)
//        imgs.push_back(cv_bridge::toCvCopy(img1_msg));
//    if (params_.num_cams >= 3)
//        imgs.push_back(cv_bridge::toCvCopy(img2_msg));
//    if (params_.num_cams >= 4)
//        imgs.push_back(cv_bridge::toCvCopy(img3_msg));
    pcl::PointCloud <PointOS1> cloud{};
    pcl::PointCloud <pcl::PointXYZRGB> rgb_cloud{};
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
    lidars_[0]->project_points(rgb_cloud, img0_msg);
    // Publish results
    // painted pc
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(rgb_cloud, msg);
    msg.header = cloud_msg->header;
    painted_pc_pub_.publish(msg);

    // sparce depth imgs
    cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
    for (int cam = 0; cam < params_.num_cams; cam++) {
        out_msg.header = img0_msg->header; // Sync time with the camera headers
        out_msg.image = lidars_[0]->virtual_cams_[cam]->img_;
        sparse_depth_pub_[cam].publish(out_msg.toImageMsg());
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
