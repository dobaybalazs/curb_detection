#include "curb_test/curb_detection.h"


CurbDetector::CurbDetector(ros::NodeHandle* nh):number_of_channels(64),number_of_points(512){
    sub_pcl = nh->subscribe(params::input_cloud,1,&CurbDetector::cloudFilter,this);

    left_lane = nh->advertise<pcl::PCLPointCloud2>("/left_points",1);
    right_lane = nh->advertise<pcl::PCLPointCloud2>("/right_points",1);
}
//First method
void CurbDetector::sortPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr converted_cloud ,const pcl::PointCloud<pcl::PointXYZI>& cloud){
    int idx = 0;
    int cur_channel = 0;

    for(int i = cloud.size() - 1; i >= 0; i--)
    {
        if ((i + 1) % 512 == 0)
            cur_channel++;

        if (cur_channel <= number_of_channels)
        {
        converted_cloud->points[idx].x = cloud.points[i].x;
        converted_cloud->points[idx].y = cloud.points[i].y;
        converted_cloud->points[idx].z = cloud.points[i].z;
        converted_cloud->points[idx].intensity = cloud.points[i].intensity;
        idx++;
        }
    }
}

double CurbDetector::euc_dist(const pcl::PointXYZI& p){
    return sqrt(pow(p.x,2)+pow(p.y,2));
}

std::vector<int> CurbDetector::intensityFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
    std::vector<int> result_indicies;    
    for(size_t i = 0;i < number_of_channels;++i){
        for(size_t j = 0;j<number_of_points-1;++j){
            float i_diff = fabs(input_cloud->points[i*number_of_points+j].intensity-input_cloud->points[i*number_of_points+j+1].intensity);
            if(input_cloud->points[i*number_of_points+j].intensity < params::max_intensity_thres 
            && input_cloud->points[i*number_of_points+j].intensity > params::min_intensity_thres 
            && i_diff < params::max_intensity_diff && i_diff > params::min_intensity_diff){
                result_indicies.push_back(i*number_of_points+j);
            }
        }
    }
    return result_indicies;
}

std::vector<int> CurbDetector::zFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
    std::vector<int> result_indicies;
    for(size_t i = 0;i < number_of_channels;++i){
        for(size_t j = 0;j<number_of_points-1;++j){
            float z_diff = fabs(input_cloud->points[i*number_of_points+j].z-input_cloud->points[i*number_of_points+j+1].z);
            if(z_diff>params::min_z_thres && z_diff<params::max_z_thres){
                result_indicies.push_back(i*number_of_points+j);
            }
        }
    }
    return result_indicies;
}

std::vector<int> CurbDetector::distFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
    std::vector<int> result_indicies;
     for(size_t i = 0;i < number_of_channels;++i){
        for(size_t j = 0;j<number_of_points-1;++j){
            double current_dist = CurbDetector::euc_dist(input_cloud->points[i*number_of_points+j]);
            double next_dist = CurbDetector::euc_dist(input_cloud->points[i*number_of_points+j+1]);
            double dist_diff = fabs(current_dist-next_dist);
            if(dist_diff>params::min_distance_thres && dist_diff<params::max_distance_thres){
                result_indicies.push_back(i*number_of_points+j);
            }
        }
    }
    return result_indicies;
}

std::vector<int> CurbDetector::angleFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
    std::vector<int> result_indicies;
     for(size_t i = 0;i < number_of_channels;++i){
        for(size_t j = 2;j<number_of_points-2;++j){
            double first_angle = atan2(input_cloud->points[i*number_of_points+j-2].y-input_cloud->points[i*number_of_points+j].y,input_cloud->points[i*number_of_points+j-2].x-input_cloud->points[i*number_of_points+j].x);
            double second_angle = atan2(input_cloud->points[i*number_of_points+j+2].y-input_cloud->points[i*number_of_points+j].y,input_cloud->points[i*number_of_points+j+2].x-input_cloud->points[i*number_of_points+j].x);
            double angle = (first_angle-second_angle)*180/M_PI;
            if (angle < 0) { angle += 2 * M_PI; }
            if(angle>params::min_angle && angle<params::max_angle){
                result_indicies.push_back(i*number_of_points+j);
            }
        }
    }
    return result_indicies;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CurbDetector::RANSACCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  // #################
  //  RANSAC
  // #################
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_points        (new pcl::PointCloud<pcl::PointXYZI>);
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_points_neg    (new pcl::PointCloud<pcl::PointXYZI>);

  // Object for Line fitting
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr      inliers      (new pcl::PointIndices ());

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
  seg.setInputCloud (cloud);                //입력 
  seg.setModelType (pcl::SACMODEL_LINE);    //적용 모델  // Configure the object to look for a plane.
  seg.setMethodType (pcl::SAC_RANSAC);      //적용 방법   // Use RANSAC method.
  seg.setMaxIterations (1000);              //최대 실행 수
  seg.setDistanceThreshold (0.1);          //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
  //seg.setRadiusLimits(0, 0.1);            // cylinder경우, Set minimum and maximum radii of the cylinder.
  seg.segment (*inliers, *coefficients);    //세그멘테이션 적용 
  
  pcl::copyPointCloud<pcl::PointXYZI> (*cloud, *inliers, *inlier_points);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (inlier_points);
  extract.setIndices (inliers);
  extract.setNegative (false);//false
  extract.filter (*inlier_points);

  return inlier_points;
}

void CurbDetector::cloudFilter(const pcl::PointCloud<pcl::PointXYZI>& cloud){
    //Sorted point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_ptr->header = cloud.header;
    //Result point
    pcl::PointCloud<pcl::PointXYZI>::Ptr result_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    result_ptr->header = cloud.header;

    pcl::PointCloud<pcl::PointXYZI>::Ptr left_curb(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr right_curb(new pcl::PointCloud<pcl::PointXYZI>);
    left_curb->header = cloud.header;
    right_curb->header = cloud.header;

    //Set cloud size
    cloud_ptr->points.resize(number_of_channels*number_of_points);

    //Sort point cloud into rings
    CurbDetector::sortPoints(cloud_ptr,cloud);

    //Declare index containers
    std::vector<int> intensity_indicies;
    std::vector<int> zdiff_indicies;
    std::vector<int> dist_indicies;
    std::vector<int> angle_indicies;

    //Result container
    std::vector<std::vector<int>> result_indicies;
    std::vector<int> conc_indicies;

    //Filter point cloud based on intensity values
    if(params::filterIntensity){
        intensity_indicies = CurbDetector::intensityFilter(cloud_ptr);
        std::sort(intensity_indicies.begin(),intensity_indicies.end());
        result_indicies.push_back(intensity_indicies);
    }
    //Filter point cloud based on distance difference values
    if(params::filterDist){
        dist_indicies = CurbDetector::distFilter(cloud_ptr);
        std::sort(dist_indicies.begin(),dist_indicies.end());
        result_indicies.push_back(dist_indicies);
    }
    //Filter point cloud based on z difference values
    if(params::filterZ){
        zdiff_indicies = CurbDetector::zFilter(cloud_ptr);
        std::sort(zdiff_indicies.begin(),zdiff_indicies.end());
        result_indicies.push_back(zdiff_indicies);
    }

    if(params::filterAngle){
        angle_indicies = CurbDetector::angleFilter(cloud_ptr);
        std::sort(angle_indicies.begin(),angle_indicies.end());
        result_indicies.push_back(angle_indicies);
    }

    //Filtering results
    auto filterCondition = boost::make_shared<FilteringCondition<pcl::PointXYZI>>(
        [=](const pcl::PointXYZI& point){
            return (CurbDetector::euc_dist(point)>2 && point.x<params::max_x && point.x>params::min_x
            && point.y<params::max_y && point.y>params::min_y && point.z<params::max_z && point.z>params::min_z);
    });
    pcl::ConditionalRemoval<pcl::PointXYZI> condition_removal;
    condition_removal.setCondition(filterCondition);
    
    if(result_indicies.empty()){
        condition_removal.setInputCloud(cloud_ptr);
        condition_removal.filter(*cloud_ptr);
        for(const auto& point:cloud_ptr->points){
            if(point.y>=0)
                left_curb->points.push_back(point);
            else
                right_curb->points.push_back(point);
        }
        //Publish results
        left_lane.publish(left_curb);
        right_lane.publish(right_curb);
    }else{
        if(result_indicies.size()==1){
            for(const auto& idx:result_indicies.front()){
                conc_indicies.push_back(idx);
            }
        }else if(result_indicies.size()==2){
            std::vector<int> intersection;
            if(params::isUnion)
                std::set_union(result_indicies.front().begin(),result_indicies.front().end(),result_indicies.back().begin(),result_indicies.back().end(),std::back_inserter(intersection));
            else
                std::set_intersection(result_indicies.front().begin(),result_indicies.front().end(),result_indicies.back().begin(),result_indicies.back().end(),std::back_inserter(intersection));
            for(const auto& idx:intersection){
                conc_indicies.push_back(idx);
            }
        }else if(result_indicies.size()==3){
            std::vector<int> temp;
            std::vector<int> intersection;
            if(params::isUnion){
                std::set_union(result_indicies.front().begin(),result_indicies.front().end(),result_indicies.back().begin(),result_indicies.back().end(),std::back_inserter(temp));
                std::set_union(temp.begin(),temp.end(),result_indicies[1].begin(),result_indicies[1].end(),std::back_inserter(intersection));
            }else{
                std::set_intersection(result_indicies.front().begin(),result_indicies.front().end(),result_indicies.back().begin(),result_indicies.back().end(),std::back_inserter(temp));
                std::set_intersection(temp.begin(),temp.end(),result_indicies[1].begin(),result_indicies[1].end(),std::back_inserter(intersection));
            }
            for(const auto& idx:intersection){
                conc_indicies.push_back(idx);
            }
        }else if(result_indicies.size()==4){
            std::vector<int> temp;
            std::vector<int> temp2;
            std::vector<int> intersection;
            if(params::isUnion){
                std::set_union(result_indicies.front().begin(),result_indicies.front().end(),result_indicies.back().begin(),result_indicies.back().end(),std::back_inserter(temp));
                std::set_union(temp.begin(),temp.end(),result_indicies[1].begin(),result_indicies[1].end(),std::back_inserter(temp2));
                std::set_union(temp2.begin(),temp2.end(),result_indicies[2].begin(),result_indicies[2].end(),std::back_inserter(intersection));
            }else{
                std::set_intersection(result_indicies.front().begin(),result_indicies.front().end(),result_indicies.back().begin(),result_indicies.back().end(),std::back_inserter(temp));
                std::set_intersection(temp.begin(),temp.end(),result_indicies[1].begin(),result_indicies[1].end(),std::back_inserter(temp2));
                std::set_intersection(temp2.begin(),temp2.end(),result_indicies[2].begin(),result_indicies[2].end(),std::back_inserter(intersection));
            }
            for(const auto& idx:intersection){
                conc_indicies.push_back(idx);
            }
        }
        for(const auto& idx:conc_indicies){
            if(cloud_ptr->points[idx].y>=0)
                left_curb->points.push_back(cloud_ptr->points[idx]);
            else
                right_curb->points.push_back(cloud_ptr->points[idx]);
        }
        condition_removal.setInputCloud(left_curb);
        condition_removal.filter(*left_curb);
        condition_removal.setInputCloud(right_curb);
        condition_removal.filter(*right_curb);
        if(params::useRansac){
            auto left = CurbDetector::RANSACCloud(left_curb);
            auto right = CurbDetector::RANSACCloud(right_curb);
            left->header = cloud.header;
            right->header = cloud.header;
            //Publish the results
            left_lane.publish(left);
            right_lane.publish(right);
            return;
        }
        left_lane.publish(left_curb);
        right_lane.publish(right_curb);
    }
}