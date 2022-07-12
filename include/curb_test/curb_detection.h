#pragma once
//ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "curb_test/curb_detectionConfig.h"
//PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

//STL includes
#include <vector>
#include <limits>
#include <algorithm>


//For pointcloud filtering
template <typename PointT>
class FilteringCondition : public pcl::ConditionBase<PointT>
{
public:
  typedef std::shared_ptr<FilteringCondition<PointT>> Ptr;
  typedef std::shared_ptr<const FilteringCondition<PointT>> ConstPtr;
  typedef std::function<bool(const PointT&)> FunctorT;

  FilteringCondition(FunctorT evaluator): 
    pcl::ConditionBase<PointT>(),_evaluator( evaluator ) 
  {}

  virtual bool evaluate (const PointT &point) const {
    // just delegate ALL the work to the injected std::function
    return _evaluator(point);
  }
private:
  FunctorT _evaluator;
};
//Parameters
namespace params{
  extern int max_intensity_thres;
  extern int min_intensity_thres;
  extern int max_intensity_diff;
  extern int min_intensity_diff;
  extern double max_z_thres;
  extern double min_z_thres;
  extern double max_distance_thres;
  extern double min_distance_thres;
  extern bool filterIntensity;
  extern bool filterZ;
  extern bool filterDist;
  extern float max_x;
  extern float min_x;
  extern float max_y;
  extern float min_y;
  extern float max_z;
  extern float min_z;
};
//Curb detector class
class CurbDetector{
    ros::Subscriber sub_pcl;
    ros::Publisher pub_fpoints;

    const size_t number_of_channels;
    const size_t number_of_points;
    
    public:

    CurbDetector(ros::NodeHandle*);
    //First method
    void sortPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr,const pcl::PointCloud<pcl::PointXYZI>&);
    
    double euc_dist(const pcl::PointXYZI&);

    std::vector<int> intensityFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr);

    std::vector<int> zFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr);

    std::vector<int> distFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr);

    void firstCloudFilter(const pcl::PointCloud<pcl::PointXYZI>&);


};