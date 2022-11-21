#include "curb_detection/curb_detector.h"

//Setting default values for parameters
bool params::filter_a=false;
bool params::filter_i=false;
bool params::filter_re=false;
bool params::filter_ra=false;
bool params::filter_z=false;
bool params::filter_dist=false;
bool params::filter_angle=false;
bool params::filter_ransac=false;
bool params::useBoxFilter=false;
float params::mult_a=1.0;
float params::mult_i=1.0;
float params::mult_ra=1.0;
float params::mult_re=1.0;
float params::mult_z=1.0;
float params::mult_angle=1.0;
float params::mult_dist=1.0;
float params::min_a=0.0;
float params::min_i=0.0;
float params::min_ra=0.0;
float params::min_re=0.0;
float params::min_zd=0.0;
float params::min_angle=0.0;
float params::min_dist=0.0;
float params::max_a=1.0;
float params::max_i=1.0;
float params::max_ra=1.0;
float params::max_re=1.0;
float params::max_zd=1.0;
float params::max_angle=1.0;
float params::max_dist=1.0;
float params::min_x = -4.0;
float params::min_y = -12.0;
float params::min_z = -1.8;
float params::max_x = 50.0;
float params::max_y = 12.0;
float params::max_z = -0.8;
float params::ransac_dist = 0.01;
float params::rradius_max = 1.0;
float params::rradius_min = 0.0;
int params::angle_wd_size = 5;
int params::dist_wd_size = 1;
float params::min_rad = 2.0;

//Setting parameter values using the server
void setParams(curb_detection::curb_detectionConfig &config,uint32_t level){
    params::filter_a = config.filter_a;
    params::filter_i = config.filter_i;
    params::filter_re = config.filter_re;
    params::filter_ra = config.filter_ra;
    params::filter_z = config.filter_z;
    params::filter_dist = config.filter_dist;
    params::filter_angle = config.filter_angle;
    params::filter_ransac = config.filter_ransac;
    params::useBoxFilter = config.useBoxFilter;
    params::max_x = config.max_x;
    params::min_x = config.min_x;
    params::max_y = config.max_y;
    params::min_y = config.min_y;
    params::max_z = config.max_z;
    params::min_z = config.min_z;
    params::mult_a = config.mult_a;
    params::mult_i = config.mult_i;
    params::mult_ra = config.mult_ra;
    params::mult_re = config.mult_re;
    params::mult_z = config.mult_z;
    params::mult_angle = config.mult_angle;
    params::mult_dist = config.mult_dist;
    params::min_a = config.min_a;
    params::min_i = config.min_i;
    params::min_ra = config.min_ra;
    params::min_re = config.min_re;
    params::min_zd = config.min_zd;
    params::min_angle = config.min_angle;
    params::min_dist = config.min_dist;
    params::max_a = config.max_a;
    params::max_i = config.max_i;
    params::max_ra = config.max_ra;
    params::max_re = config.max_re;
    params::max_zd = config.max_zd;
    params::max_angle = config.max_angle;
    params::max_dist = config.max_dist;
    params::angle_wd_size = config.angle_wd_size;
    params::dist_wd_size = config.dist_wd_size;
    params::ransac_dist = config.ransac_dist;
    params::rradius_max = config.rradius_max;
    params::rradius_min = config.rradius_min;
    params::min_rad = config.min_rad;
}

int main(int argc,char** argv){
    ros::init(argc,argv,"curb_detection_node");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    //Setting up the paramter server
    dynamic_reconfigure::Server<curb_detection::curb_detectionConfig> server;
    dynamic_reconfigure::Server<curb_detection::curb_detectionConfig>::CallbackType f;

    f = boost::bind(&setParams,_1,_2);
    server.setCallback(f); 

    //Creating the curb detection instance
    CurbDetector cd(nh);

    ros::spin();
}