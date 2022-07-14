#include "curb_test/curb_detection.h"

int params::max_intensity_thres = 1000;
int params::min_intensity_thres = 0;
int params::min_intensity_diff = 0;
int params::max_intensity_diff = 100;
double params::max_z_thres = 1.0;
double params::min_z_thres = 0.0;
double params::max_distance_thres = 1.0;
double params::min_distance_thres = 0.0;
bool params::filterIntensity = false;
bool params::filterZ = false;
bool params::filterDist = false;
float params::max_x = 5;
float params::min_x = 5;
float params::max_y = 5;
float params::min_y = 5;
float params::max_z = 5;
float params::min_z = 5;
bool params::isUnion = false;
bool params::filterAngle = false;
double params::max_angle = 180.0;
double params::min_angle = 0.0;

void setParams(curb_test::curb_detectionConfig &config,uint32_t level){
    params::max_intensity_thres = config.max_intensity;
    params::min_intensity_thres = config.min_intensity;
    params::min_intensity_diff = config.min_intensity_diff;
    params::max_intensity_diff = config.max_intensity_diff;
    params::max_z_thres= config.z_max;
    params::min_z_thres= config.z_min;
    params::max_distance_thres = config.distance_max;
    params::min_distance_thres = config.distance_min;
    params::filterIntensity = config.filterIntensity;
    params::filterZ = config.filterZ;
    params::filterDist = config.filterDist;
    params::max_x = config.max_x;
    params::min_x = config.min_x;
    params::max_y = config.max_y;
    params::min_y = config.min_y;
    params::max_z = config.max_z;
    params::min_z = config.min_z;
    params::isUnion = config.isUnion;
    params::filterAngle = config.filterAngle;
    params::max_angle = config.angle_max;
    params::min_angle = config.angle_min;
}


int main(int argc,char** argv){
    ros::init(argc,argv,"curb_detection");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<curb_test::curb_detectionConfig> server;
    dynamic_reconfigure::Server<curb_test::curb_detectionConfig>::CallbackType f;

    f = boost::bind(&setParams,_1,_2);
    server.setCallback(f); 

    CurbDetector cb(&nh);

    ros::spin();
}

