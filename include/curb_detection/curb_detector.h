#pragma once

//ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/server.h>

#include "curb_detection/curb_detectionConfig.h"
//ouster
#include "./point.h"
//pcl
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/method_types.h>

//stl
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <string>

namespace params{
    extern bool filter_a,filter_i,filter_re,filter_ra,filter_z,filter_dist,filter_angle, filter_ransac, useBoxFilter;
    extern float mult_a,mult_i,mult_ra,mult_re,mult_z,mult_angle,mult_dist;
    extern float min_a,min_i,min_ra,min_re,min_zd,min_angle,min_dist;
    extern float max_a,max_i,max_ra,max_re,max_zd,max_angle,max_dist;
    extern float min_x,min_y,min_z;
    extern float max_x,max_y,max_z;
    extern float min_rad;
    extern int angle_wd_size,dist_wd_size;
    extern float ransac_dist,rradius_min,rradius_max;
};

class CurbDetector{
    
    ros::Subscriber sub;
    ros::Publisher pub_right;
    ros::Publisher pub_left;
    
    std::string input_topic;
    int number_of_points;
    int number_of_channels;

    public:
    CurbDetector(ros::NodeHandlePtr);

    void RANSACCloud(pcl::PointCloud<ouster::Point>::Ptr);

    void sortPoints(pcl::PointCloud<ouster::Point>::Ptr,const pcl::PointCloud<ouster::Point>::ConstPtr);

    void limitFilter(pcl::PointCloud<ouster::Point>::Ptr);

    void boxFilter(const pcl::PointCloud<ouster::Point>::ConstPtr,pcl::PointCloud<ouster::Point>::Ptr);

    void callBack(const pcl::PointCloud<ouster::Point>::ConstPtr);
};