#include "curb_detection/curb_detector.h"


CurbDetector::CurbDetector(ros::NodeHandlePtr nh):number_of_channels(64),number_of_points(512),input_topic("/left_os1/os1_cloud_node/points"){
    nh->getParam("point_number",number_of_points);
    nh->getParam("channel_number",number_of_channels);
    nh->getParam("input_topic",input_topic);
       
    sub = nh->subscribe(input_topic,1,&CurbDetector::callBack,this);

    pub = nh->advertise<pcl::PCLPointCloud2>("/output",1);
}

void CurbDetector::sortPoints(pcl::PointCloud<ouster_ros::Point>::Ptr converted_cloud,const pcl::PointCloud<ouster_ros::Point>::ConstPtr cloud){
    int idx = 0;
    int cur_channel = 0;

    for(int i = number_of_channels*number_of_points - 1; i >= 0; i--)
    {
        if ((i + 1) % number_of_points == 0)
            cur_channel++;

        if (cur_channel <= number_of_channels)
        {
        converted_cloud->points[idx].x = cloud->points[i].x;
        converted_cloud->points[idx].y = cloud->points[i].y;
        converted_cloud->points[idx].z = cloud->points[i].z;
        converted_cloud->points[idx].intensity = cloud->points[i].intensity;
        converted_cloud->points[idx].ambient = cloud->points[i].ambient;
        converted_cloud->points[idx].reflectivity = cloud->points[i].reflectivity;
        converted_cloud->points[idx].range = cloud->points[i].range;
        idx++;
        }
    }
}

void CurbDetector::limitFilter(pcl::PointCloud<ouster_ros::Point>::Ptr cloud){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());

    pcl::ExtractIndices<ouster_ros::Point> extract;
    std::unordered_set<int> indices;
    for(size_t i=0;i<number_of_channels;++i){
        float max_diff_z=0,max_int=0,max_dist=0,max_angle=0,max_refl=0,max_amb=0;
        u_int32_t max_rang=0;
        for(size_t j=1;j<number_of_points-1;++j){
            float current_z_diff = std::fabs(cloud->points[i*number_of_points+j].z-cloud->points[i*number_of_points+j-1].z);
            max_diff_z = std::max(max_diff_z,current_z_diff);
            float current_int_diff = std::fabs(cloud->points[i*number_of_points+j+1].intensity - cloud->points[i*number_of_points+j].intensity);
            max_int = std::max(max_int,current_int_diff);
            float current_refl_diff = std::fabs(cloud->points[i*number_of_points+j+1].reflectivity - cloud->points[i*number_of_points+j].reflectivity);
            max_refl = std::max(max_refl,current_refl_diff);
            max_rang = std::max(max_rang,cloud->points[i*number_of_points+j].range);
            float current_amb_diff = std::fabs(cloud->points[i*number_of_points+j+1].ambient - cloud->points[i*number_of_points+j].ambient);
            max_amb = std::max(max_amb,current_amb_diff);
            float current_dist = std::sqrt(std::pow(cloud->points[i*number_of_points+j].x,2)+std::pow(cloud->points[i*number_of_points+j].y,2));
            max_dist = std::max(max_dist,current_dist);
            float x1 = cloud->points[i*number_of_points+j-1].x - cloud->points[i*number_of_points+j].x;
            float x2 = cloud->points[i*number_of_points+j+1].x - cloud->points[i*number_of_points+j].x;
            float y1 = cloud->points[i*number_of_points+j-1].y - cloud->points[i*number_of_points+j].y;
            float y2 = cloud->points[i*number_of_points+j+1].y - cloud->points[i*number_of_points+j].y;
            float dot_prod = x1*x2+y1*y2;
            float det = x1*y2-y1*x2;
            float current_angle = atan2(det,dot_prod);
            max_angle = std::max(max_angle,current_angle);
        }
        for(size_t j=1;j<number_of_points-1;++j){
            if(params::filter_a){
                float amb_val = std::abs(cloud->points[i*number_of_points+j+1].ambient*params::mult_a - cloud->points[i*number_of_points+j].ambient*params::mult_a);
                if(amb_val>max_amb*params::min_a && amb_val<max_amb*params::max_a){
                    indices.insert(i*number_of_points+j);
                    indices.insert(i*number_of_points+j+1);
                }
            }
            if(params::filter_i){
                float int_val = std::fabs(cloud->points[i*number_of_points+j+1].intensity*params::mult_i - cloud->points[i*number_of_points+j].intensity*params::mult_i);
                if(int_val>max_int*params::min_i && int_val<max_int*params::max_i){
                    indices.insert(i*number_of_points+j);
                    indices.insert(i*number_of_points+j+1);
                }
            }
            if(params::filter_ra){
                float range_val = static_cast<float>(cloud->points[i*number_of_points+j].range)*params::mult_ra;
                if(range_val>max_rang*params::min_ra && range_val<max_rang*params::max_ra){
                    indices.insert(i*number_of_points+j);
                    indices.insert(i*number_of_points+j+1);
                }
            }
            if(params::filter_re){
                float ref_val = std::abs(cloud->points[i*number_of_points+j+1].reflectivity*params::mult_re - cloud->points[i*number_of_points+j].reflectivity*params::mult_re);
                if(ref_val>max_refl*params::min_re && ref_val<max_refl*params::max_re){
                    indices.insert(i*number_of_points+j);
                    indices.insert(i*number_of_points+j+1);
                }
            }
            if(params::filter_z){
                double zdiff_val = std::fabs(cloud->points[i*number_of_points+j].z-cloud->points[i*number_of_points+j-1].z);
                if(zdiff_val>max_diff_z*params::min_zd && zdiff_val<max_diff_z*params::max_zd){
                    indices.insert(i*number_of_points+j);
                    indices.insert(i*number_of_points+j+1);
                }
            }
        }
        if(params::filter_dist){
            int wd = params::dist_wd_size;
            for(size_t j=wd;j<number_of_points-wd;++j){
                double dist_prev = std::sqrt(std::pow(cloud->points[i*number_of_points+j-wd].x,2)+std::pow(cloud->points[i*number_of_points+j-wd].y,2))*params::mult_dist;
                double dist_next = std::sqrt(std::pow(cloud->points[i*number_of_points+j+wd].x,2)+std::pow(cloud->points[i*number_of_points+j+wd].y,2))*params::mult_dist;
                double dist_val = std::fabs(dist_prev-dist_next)*params::mult_dist;
                if(dist_val>max_dist*params::min_dist && dist_val<max_dist*params::max_dist){
                    indices.insert(i*number_of_points+j);
                    indices.insert(i*number_of_points+j+1);
                }
            }
        }
        if(params::filter_angle){
            int wd = params::angle_wd_size;
            for(size_t j=wd;j<number_of_points-wd;++j){
                double x1 = cloud->points[i*number_of_points+j-wd].x - cloud->points[i*number_of_points+j].x;
                double x2 = cloud->points[i*number_of_points+j+wd].x - cloud->points[i*number_of_points+j].x;
                double y1 = cloud->points[i*number_of_points+j-wd].y - cloud->points[i*number_of_points+j].y;
                double y2 = cloud->points[i*number_of_points+j+wd].y - cloud->points[i*number_of_points+j].y;
                double dot_prod = x1*x2+y1*y2;
                double det = x1*y2-y1*x2;
                double angle_val = atan2(det,dot_prod)*params::mult_angle;
                if(angle_val>max_angle*params::min_angle && angle_val<max_angle*params::max_angle){
                    indices.insert(i*number_of_points+j);
                    indices.insert(i*number_of_points+j+1);
                }
            }
        }
    }
    if(!indices.empty()){
        inliers->indices.assign(indices.begin(),indices.end());
        extract.setIndices(inliers);
        extract.filterDirectly(cloud);
    }
}

void CurbDetector::boxFilter(const pcl::PointCloud<ouster_ros::Point>::ConstPtr input_cloud,pcl::PointCloud<ouster_ros::Point>::Ptr output_cloud){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    pcl::CropBox<ouster_ros::Point> roi(true);

    Eigen::Vector4f min(params::min_x,params::min_y,params::min_z,0.0);
    Eigen::Vector4f max(params::max_x,params::max_y,params::max_z,0.0);

    roi.setMin(min);
    roi.setMax(max);
    roi.setInputCloud(input_cloud);
    roi.filter(inliers->indices);

    pcl::ExtractIndices<ouster_ros::Point> extract;
    extract.setIndices(inliers);
    extract.setInputCloud(input_cloud);
    extract.filter(*output_cloud);
}

void CurbDetector::callBack(const pcl::PointCloud<ouster_ros::Point>::ConstPtr cloud){
    pcl::PointCloud<ouster_ros::Point>::Ptr temp(new pcl::PointCloud<ouster_ros::Point>());
    pcl::PointCloud<ouster_ros::Point>::Ptr result(new pcl::PointCloud<ouster_ros::Point>());

    temp->header = cloud->header;
    temp->points.resize(number_of_channels*number_of_points);
    //Sort cloud_points
    CurbDetector::sortPoints(temp,cloud);
    //Filter cloud based on certain characteristics
    CurbDetector::limitFilter(temp);
    //Filter cloud based on x,y and z values
    CurbDetector::boxFilter(temp,result);
    result->header = cloud->header;
    pub.publish(result);
}