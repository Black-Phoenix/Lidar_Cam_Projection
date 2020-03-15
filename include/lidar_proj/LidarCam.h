//
// Created by raven on 3/12/20.
//

#ifndef LIDAR_PROJ_LIDARCAM_H
#define LIDAR_PROJ_LIDARCAM_H

#include <ros/ros.h>
#include "lidar_proj/Camera.h"
#include "lidar_proj/Lidar.h"
#include <string.h>
#include <vector>
#include "Lidar.h"
#include "Camera.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

using namespace std;
namespace lidar_proj {

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> policy_single;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image> policy_dual;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> policy_triple;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> policy_quad;

    class LidarCam {
    public:
        LidarCam(ros::NodeHandle &nh) : nh_(nh) {};

        void init();

        void cam_lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &,
                                const sensor_msgs::Image::ConstPtr &,
                                const sensor_msgs::Image::ConstPtr &,
                                const sensor_msgs::Image::ConstPtr &,
                                const sensor_msgs::Image::ConstPtr &);

        ~LidarCam();

        void project_points(pcl::PointCloud<pcl::PointXYZRGB> &rgb_cloud,
                            const vector<cv_bridge::CvImageConstPtr> &rgb_imgs);

    private:
        Params params_;
//        vector<Camera *> cams_;
        vector<Lidar *> lidars_;
        // Publishers & ROS node
        ros::NodeHandle nh_;
        ros::Publisher painted_pc_pub_;
        vector<ros::Publisher> sparse_depth_pub_; // Because we can have multiple cams
        // Subscribers
        image_transport::ImageTransport *it_; // compressed images
        message_filters::Subscriber<sensor_msgs::PointCloud2> *lidar_sub_;
        std::vector<image_transport::SubscriberFilter *> img_subs_;

        // Sync policies. Not sure of a better way
        message_filters::Synchronizer<policy_single> *sync_single_;
        message_filters::Synchronizer<policy_dual> *sync_dual_;
        message_filters::Synchronizer<policy_triple> *sync_triple_;
        message_filters::Synchronizer<policy_quad> *sync_quad_;
    };
}

#endif //LIDAR_PROJ_LIDARCAM_H
