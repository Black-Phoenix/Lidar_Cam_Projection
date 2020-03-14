//
// Created by raven on 3/12/20.
//
#include <ros/ros.h>
#include <iostream>
#include "lidar_proj/LidarCam.h"
using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_proj");
    ros::NodeHandle nh("~");
    try{
        lidar_proj::LidarCam node(nh);
        node.init();
        ros::spin();
    } catch (std::exception& e){
        ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    }
    return 0;
}

