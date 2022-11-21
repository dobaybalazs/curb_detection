#include "curb_detection/curb_detector.h"

//Setting default values for params
CurbDetector::CurbDetector(ros::NodeHandlePtr nh){
    nh->getParam("point_number",number_of_points);
    nh->getParam("channel_number",number_of_channels);
    nh->getParam("input_topic",input_topic);
       
    sub = nh->subscribe(input_topic,1,&CurbDetector::callBack,this);

    pub_right = nh->advertise<pcl::PCLPointCloud2>("/left_points",1);
    pub_left = nh->advertise<pcl::PCLPointCloud2>("/right_points",1);
}

//Function for noise filtering using RANSAC
void CurbDetector::RANSACCloud(pcl::PointCloud<ouster::Point>::Ptr cloud)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr      inliers      (new pcl::PointIndices ());

    pcl::SACSegmentation<ouster::Point> seg;
    seg.setOptimizeCoefficients (true);       
    seg.setInputCloud (cloud);                
    seg.setModelType (pcl::SACMODEL_LINE);    
    seg.setMethodType (pcl::SAC_RANSAC);      
    seg.setMaxIterations (1000);              
    seg.setDistanceThreshold (params::ransac_dist);       
    seg.setRadiusLimits(params::rradius_min, params::rradius_max);            
    seg.segment (*inliers, *coefficients);    
    
    pcl::ExtractIndices<ouster::Point> extract;

    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filterDirectly(cloud);
}

//Sorting condition ascending order
bool comp_up(const ouster::Point &a, const ouster::Point &b)
{
    return a.y < b.y;
}
//Sorting condition descending order
bool comp_down(const ouster::Point &a, const ouster::Point &b)
{
    return a.y > b.y;
}
//Function for filtering based on threshold values
void filterPoints(pcl::PointCloud<ouster::Point>::Ptr cloud,const std::unordered_map<std::string,float>& limits,pcl::PointCloud<ouster::Point>::Ptr output){
    bool f_a = true;
    bool f_i = true;
    bool f_ra = true;
    bool f_re = true;
    bool f_z = true;
    if(cloud->points.size()>2){
        for(size_t j=1;j<cloud->points.size()-1;++j){
            int a = f_a +f_i +f_ra+f_re+f_z;
            int b = params::filter_a+params::filter_i+params::filter_ra+params::filter_re+params::filter_z;
            if(params::filter_a && f_a){
                float max_amb=limits.at("max_amb");
                float amb_val = std::abs(cloud->points[j+1].ambient*params::mult_a - cloud->points[j].ambient*params::mult_a);
                if(amb_val>max_amb*params::min_a && amb_val<max_amb*params::max_a){
                    output->points.push_back(cloud->points[j]);
                    f_a = false;
                }
            }
            if(params::filter_i && f_i){
                float max_int=limits.at("max_int");
                float int_val = std::fabs(cloud->points[j+1].intensity*params::mult_i - cloud->points[j].intensity*params::mult_i);
                if(int_val>max_int*params::min_i && int_val<max_int*params::max_i){
                    output->points.push_back(cloud->points[j]);
                    f_i = false;
                }
            }
            if(params::filter_ra && f_ra){
                float max_rang=limits.at("max_rang");
                float range_val = static_cast<float>(cloud->points[j].range)*params::mult_ra;
                if(range_val>max_rang*params::min_ra && range_val<max_rang*params::max_ra){
                    output->points.push_back(cloud->points[j]);
                    f_ra = false;
                }
            }
            if(params::filter_re && f_re){
                float max_refl=limits.at("max_refl");
                float ref_val = std::abs(cloud->points[j+1].reflectivity*params::mult_re - cloud->points[j].reflectivity*params::mult_re);
                if(ref_val>max_refl*params::min_re && ref_val<max_refl*params::max_re){
                    output->points.push_back(cloud->points[j]);
                    f_re = false;
                }
            }
            if(params::filter_z && f_z){
                double zdiff_val = std::fabs(cloud->points[j].z-cloud->points[j+1].z);
                if(zdiff_val>params::min_zd && zdiff_val<params::max_zd){
                    output->points.push_back(cloud->points[j]);
                    f_z = false;
                }
            }
            if(a==b) break;
        }
    }
    if(params::filter_dist){
        float max_dist=limits.at("max_dist");
        int wd = params::dist_wd_size;
        if(wd<cloud->points.size() && cloud->points.size()-wd>wd){
            for(size_t j=wd;j<cloud->points.size()-wd;++j){
                double dist_prev = std::sqrt(std::pow(cloud->points[j-wd].x,2)+std::pow(cloud->points[j-wd].y,2))*params::mult_dist;
                double dist_next = std::sqrt(std::pow(cloud->points[j+wd].x,2)+std::pow(cloud->points[j+wd].y,2))*params::mult_dist;
                double dist_val = std::fabs(dist_prev-dist_next)*params::mult_dist;
                if(dist_val>max_dist*params::min_dist && dist_val<max_dist*params::max_dist){
                    output->points.push_back(cloud->points[j]);
                    break;
                }
            }
        }
    }
    if(params::filter_angle){
        float max_angle=limits.at("max_angle");
        int wd = params::angle_wd_size;
        if(wd<cloud->points.size() && cloud->points.size()-wd>wd){
            for(size_t j=wd;j<cloud->points.size()-wd;++j){
                double x1 = cloud->points[j-wd].x - cloud->points[j].x;
                double x2 = cloud->points[j+wd].x - cloud->points[j].x;
                double y1 = cloud->points[j-wd].y - cloud->points[j].y;
                double y2 = cloud->points[j+wd].y - cloud->points[j].y;
                double dot_prod = x1*x2+y1*y2;
                double det = x1*y2-y1*x2;
                double angle_val = atan2(det,dot_prod)*params::mult_angle;
                if(angle_val>max_angle*params::min_angle && angle_val<max_angle*params::max_angle){
                    output->points.push_back(cloud->points[j]);
                    break;
                }
            }
        }
    }
}
//Function for setting the threshold values
void CurbDetector::limitFilter(const pcl::PointCloud<ouster::Point>::ConstPtr cloud,pcl::PointCloud<ouster::Point>::Ptr left,pcl::PointCloud<ouster::Point>::Ptr right){
    for(size_t i=0;i<number_of_channels;++i){
        std::unordered_map<std::string,float> limit_map;
        float max_int=0,max_dist=0,max_angle=0,max_refl=0,max_amb=0;
        u_int32_t max_rang=0;
        for(size_t j=1;j<number_of_points-1;++j){
            if(params::filter_i){
                float current_int_diff = std::fabs(cloud->points[i*number_of_points+j+1].intensity - cloud->points[i*number_of_points+j].intensity);
                max_int = std::max(max_int,current_int_diff);
            }
            if(params::filter_re){
                float current_refl_diff = std::fabs(cloud->points[i*number_of_points+j+1].reflectivity - cloud->points[i*number_of_points+j].reflectivity);
                max_refl = std::max(max_refl,current_refl_diff);
            }
            if(params::filter_ra)
                max_rang = std::max(max_rang,cloud->points[i*number_of_points+j].range);
            if(params::filter_a){
                float current_amb_diff = std::fabs(cloud->points[i*number_of_points+j+1].ambient - cloud->points[i*number_of_points+j].ambient);
                max_amb = std::max(max_amb,current_amb_diff);
            }
            if(params::filter_dist){
                float current_dist = std::sqrt(std::pow(cloud->points[i*number_of_points+j].x,2)+std::pow(cloud->points[i*number_of_points+j].y,2));
                max_dist = std::max(max_dist,current_dist);
            }
            if(params::filter_angle){
                float x1 = cloud->points[i*number_of_points+j-1].x - cloud->points[i*number_of_points+j].x;
                float x2 = cloud->points[i*number_of_points+j+1].x - cloud->points[i*number_of_points+j].x;
                float y1 = cloud->points[i*number_of_points+j-1].y - cloud->points[i*number_of_points+j].y;
                float y2 = cloud->points[i*number_of_points+j+1].y - cloud->points[i*number_of_points+j].y;
                float dot_prod = x1*x2+y1*y2;
                float det = x1*y2-y1*x2;
                float current_angle = atan2(det,dot_prod);
                max_angle = std::max(max_angle,current_angle);
            }
        }
        if(params::filter_i)
            limit_map["max_int"] = max_int;
        if(params::filter_dist)
            limit_map["max_dist"] = max_dist;
        if(params::filter_angle)
            limit_map["max_angle"] = max_angle;
        if(params::filter_re)
            limit_map["max_refl"] = max_refl;
        if(params::filter_a)
            limit_map["max_amb"] = max_amb;
        if(params::filter_ra)
            limit_map["max_rang"] = static_cast<float>(max_rang);

        pcl::PointCloud<ouster::Point>::Ptr pc_left(new pcl::PointCloud<ouster::Point>);
        pcl::PointCloud<ouster::Point>::Ptr pc_right(new pcl::PointCloud<ouster::Point>);
        for(size_t j=0;j<number_of_points;++j){
            auto current_point = cloud->points[i*number_of_points+j];
            if(std::sqrt(std::pow(current_point.x,2)+std::pow(current_point.y,2))>params::min_rad){
                if(current_point.y >= 0){
                    pc_left->push_back(current_point);
                }else{
                    pc_right->push_back(current_point);
                }
            }
        }
        std::sort(pc_left->begin(),pc_left->end(),comp_up);
        std::sort(pc_right->begin(),pc_right->end(),comp_down);
        filterPoints(pc_left,limit_map,left);
        filterPoints(pc_right,limit_map,right);
    }
}
//function for box-filtering the cloud
void CurbDetector::boxFilter(const pcl::PointCloud<ouster::Point>::ConstPtr input_cloud,pcl::PointCloud<ouster::Point>::Ptr output_cloud){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    pcl::CropBox<ouster::Point> roi(true);

    Eigen::Vector4f min(params::min_x,params::min_y,params::min_z,0.0);
    Eigen::Vector4f max(params::max_x,params::max_y,params::max_z,0.0);

    roi.setMin(min);
    roi.setMax(max);
    roi.setInputCloud(input_cloud);
    roi.filter(inliers->indices);

    pcl::ExtractIndices<ouster::Point> extract;
    extract.setIndices(inliers);
    extract.setInputCloud(input_cloud);
    extract.filter(*output_cloud);
}
//input data callback
void CurbDetector::callBack(const pcl::PointCloud<ouster::Point>::ConstPtr cloud){
    pcl::PointCloud<ouster::Point>::Ptr right(new pcl::PointCloud<ouster::Point>());
    pcl::PointCloud<ouster::Point>::Ptr left(new pcl::PointCloud<ouster::Point>());
    pcl::PointCloud<ouster::Point>::Ptr res_right(new pcl::PointCloud<ouster::Point>());
    pcl::PointCloud<ouster::Point>::Ptr res_left(new pcl::PointCloud<ouster::Point>());
    //Filter cloud based on certain characteristics
    CurbDetector::limitFilter(cloud,left,right);
    //Filter cloud based on x,y and z values
    if(params::useBoxFilter){
        CurbDetector::boxFilter(left,res_left);
        CurbDetector::boxFilter(right,res_right);
    }else{
        res_left->points = left->points;
        res_right->points = right->points;
    }

    res_left->header = cloud->header;
    res_right->header = cloud->header;
    //Use RANSAC on filtered clouds
    if(params::filter_ransac){
        CurbDetector::RANSACCloud(res_left);
        CurbDetector::RANSACCloud(res_right);
    }
    pub_left.publish(res_left);
    pub_right.publish(res_right);
}