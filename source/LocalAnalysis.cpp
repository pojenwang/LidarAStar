#include "LidarAStar.hpp"
#include "LocalAnalysis.hpp"

#define OBSTACLE_CELL_THRESHOLD     30

LocalAnalysis::LidarScan LocalAnalysis::LidarSim::simulateLidarScan2(cv::Mat map_, Vector3d scan_pose_, float range_, int num_data_){
    float scan_heading_cart = 90 - scan_pose_.z;
    scan_heading_cart = (scan_heading_cart < 0 ? scan_heading_cart + 360 : scan_heading_cart);
    //std::cout<<"\nscan_head_cart:"<<scan_heading_cart<<"";
    float angular_sector = 180/(num_data_-1);
    vector<float> angles;
    vector<float> ranges;
    vector<int> labels;
    for(int i = 0; i < num_data_; i++){
        angles.push_back(angular_sector * i);
        ranges.push_back(1000);
        labels.push_back(0);
        float ray_dis = 1;
        bool obstacle_cell_found = false;
        while(ray_dis <= range_ * PIX_PER_METER && !obstacle_cell_found){
            int x = scan_pose_.x* PIX_PER_METER + ray_dis * cos((scan_heading_cart - 90 + i * angular_sector) * PI/180);
            int y = scan_pose_.y* PIX_PER_METER - ray_dis * sin((scan_heading_cart - 90 + i * angular_sector) * PI/180);
            //std::cout<<" (x:"<<x<<" y:"<<y<<")";
            if(map_.at<uchar>(y,x) > OBSTACLE_CELL_THRESHOLD){
                ranges[i] = ray_dis/PIX_PER_METER;
                obstacle_cell_found = true;
            }
            ray_dis++;
        }
    }
    LidarScan scan;
    scan.scan_pose = scan_pose_;
    scan.detection_range = range_;
    scan.angles = angles;
    scan.ranges = ranges;
    scan.labels = labels;
    scan.num_data = num_data_;

    return scan;
}

void LocalAnalysis::LidarSim::printLidarScan(LocalAnalysis::LidarScan scan_){
    std::cout<<"Scan:{Angle, Range, Label}:\n";
    for(uint i = 0; i< scan_.angles.size();i++){
        std::cout<<"{"<<scan_.angles[i]<<", "<<scan_.ranges[i]<<", "<<scan_.labels[i]<<"},";
    }
    std::cout<<"\n";
}


vector<pair<Vector2d,Vector2d>> LocalAnalysis::ScanAnalysis::getGatewaysFromScan(LidarScan& scan_, 
                                        pair<Vector2d,Vector2d> edge_pts, float grouping_radius_, float valid_range_){
    scan_.grouping_radius = grouping_radius_;
    //float edge_pt_dis = sqrt(pow(edge_pts.first.x()- edge_pts.second.x(),2) + pow(edge_pts.first.y() - edge_pts.second.y(),2));
    
    //insert dummy obstacle data at 0 and 180 degree to potentially create gateways with the first and last obstacle clusters
    float gateway_length = sqrt(pow(edge_pts.first.x - edge_pts.second.x,2) + pow(edge_pts.first.y - edge_pts.second.y,2));
    
    //std::cout<<"\ngateway_length:"<<gateway_length;
    
    //scan_.ranges[0] = min(scan_.ranges[0], grouping_radius_ / 2 + (float)0.1);
    //scan_.ranges[scan_.num_data-1] = min(scan_.ranges[scan_.num_data-1], grouping_radius_ / 2 + (float)0.1);
    
    if(scan_.ranges[0] == 1000)
        scan_.ranges[0] = gateway_length/2;
    if(scan_.ranges[scan_.num_data-1] == 1000)
        scan_.ranges[scan_.num_data-1] = gateway_length/2;
    
//------------cluster scan data into groups that have distacnes smaller or equal to grouping_radius ------
    
    uint new_cluster_index = 1;
    for(int i = 0; i < scan_.num_data; ++i){
        if(scan_.ranges[i] < valid_range_){
            if(scan_.labels[i] == 0){
                scan_.labels[i] = new_cluster_index;
                new_cluster_index++;
            }
            int max_dir_to_check;
            if(grouping_radius_ < scan_.ranges[i]){
                max_dir_to_check = scan_.angles[i] + ceil(asin((float)grouping_radius_/scan_.ranges[i]) * 180 / PI);
                max_dir_to_check = max_dir_to_check > 180 ? 180 : max_dir_to_check;
            }else{
                max_dir_to_check = 180;
            }
           
            int index_check_limit = ceil(max_dir_to_check * (scan_.num_data-1)/180);
            index_check_limit = index_check_limit > (scan_.num_data-1) ? (scan_.num_data-1):index_check_limit;

            //cout<<"\n(min,max):("<<minDirToCheck<<","<<maxDirToCheck<<")\n";
            for(int j = i+1; j <= index_check_limit; j++){
                //std::cout<<"data dis:("<<i<<","<<j<<","<<ScanDataDistance(scan_, i, j)<<") ScanRange:"<<scan_.ranges[j]
                //<<" Label:("<<scan_.labels[i]<<","<<scan_.labels[j]<<")\n";
                             
                if(scan_.ranges[j] < valid_range_ && scan_.labels[i] != scan_.labels[j] && 
                    ScanDataDistance(scan_, i, j) <= grouping_radius_){
                    if(scan_.labels[j] == 0){
                        
                        scan_.labels[j] = scan_.labels[i];
                    }else{
                        //
                        for(int k = 0; k <= index_check_limit; k++){
                            if(scan_.labels[k] == scan_.labels[i]){
                                scan_.labels[k] = scan_.labels[j]; //merge to the larger number cluster label
                            }
                        }
                    }
                }
            }
        }
    }    
    
    int max_cluster_label = 0;
    //check the maximum label number ignoring the dummy cluster at 0 and 180 degree
    for(int i = 1; i < scan_.num_data-1; i++){
        if(scan_.labels[i] > max_cluster_label)
            max_cluster_label = scan_.labels[i];
    }
    scan_.no_obstacle_error = (max_cluster_label == 0);

    //reassign cluster labels such that they will not skip any number
    max_cluster_label = 0;
    for(int i = 0; i < scan_.num_data; i++){
        if(scan_.labels[i] > max_cluster_label)
            max_cluster_label = scan_.labels[i];
    }    
    
    int new_cluster_label = 1;
    for(int i = 1; i <= max_cluster_label; i++){
        bool index_found = false;
        for(int j = 0; j <= scan_.num_data; j++){
            if(scan_.labels[j] == i){
                scan_.labels[j] = new_cluster_label;
                index_found = true;
            }
        }
        if(index_found){
            new_cluster_label++;
        }
    }    
    int num_cluster = new_cluster_label - 1;
    scan_.num_cluster = num_cluster;
    
    //std::cout<<"\nclusters:";
    //find the right(min angle) and left(max angle) cluster edges
    for(int i = 1; i <= num_cluster; i++){
        scan_.cluster_edges.push_back({0,0});//initialization for each cluster
        bool right_edge_found = false;
        for(int j = 0; j < scan_.num_data; j++){
            if(scan_.labels[j] == i){               
                if(!right_edge_found){
                    right_edge_found = true;
                    scan_.cluster_edges[i-1].first = j;//set right edge
                }
                scan_.cluster_edges[i-1].second = j; //set left edge
            }
        }
    }
    /*    
    for(int i = 0; i<num_cluster; i++){
        std::cout<<"{"<<scan_.cluster_edges[i].first<<","<<scan_.cluster_edges[i].second<<"}";
    }*/


//-----------------------------remove clusters entirely behind another cluster-------------------------------

    float cluster_min_dis[num_cluster];
    std::fill_n(cluster_min_dis, num_cluster + 1, 1000);//cluster_min_dis[0] not used
    
    //find the minimum distance within each cluster
    for(int i = 0; i < scan_.num_data; i++){
        if(scan_.ranges[i] < cluster_min_dis[scan_.labels[i]]){
            cluster_min_dis[scan_.labels[i]] = scan_.ranges[i];
        }
    }
    //std::cout<<"\ncluster_min_dis:";
    /*
    for(int i =0; i<num_cluster; i++){
        std::cout<<cluster_min_dis[i]<<", ";
    */
    /*
    check for every possible combination of two clusters, if one is behind the other
    (both the left and right edges are within the left and right edges of another cluster),
    the rear cluster is removed
    */
   
    for(int i = 0; i < num_cluster; i++){
        for(int j = 0; j < num_cluster; j++){
            if(i != j && (scan_.cluster_edges[i].first > scan_.cluster_edges[j].first && 
                        scan_.cluster_edges[i].second < scan_.cluster_edges[j].second)){
                if(cluster_min_dis[scan_.labels[scan_.cluster_edges[i].first]] < 
                    cluster_min_dis[scan_.labels[scan_.cluster_edges[j].first]]){
                    scan_.cluster_edges.erase(scan_.cluster_edges.begin() + j);
                    //std::cout<<"\ncluster "<<j<<" removed!";
                }else{
                    scan_.cluster_edges.erase(scan_.cluster_edges.begin() + i);
                    //std::cout<<"\ncluster "<<i<<" removed!";
                }
            }
        }
    } 


    num_cluster = scan_.cluster_edges.size();
    scan_.num_cluster = scan_.cluster_edges.size();


//-------------------------------------determine gateways given the cluster edges-----------------------------------------
   
    //convert from compass heading to cartesian heading
    float scan_heading_cart = 90 - scan_.scan_pose.z;
    scan_heading_cart = (scan_heading_cart < 0 ? scan_heading_cart + 360 : scan_heading_cart);

    //insert gateways between the edges of two neighboring clusters, calculate their global positions on the map
    for(int i = 0; i < num_cluster - 1; i++){
        // determine the cluster edges' global positions: 
        // X = X' + d * cos(theta' + theta)
        // Y = Y' - d * sin(theta' + theta)
        // X' and theta' are the scan's global pose, d and theta are local scan's ranges and directions
        const float g_right_bound_x = scan_.scan_pose.x + scan_.ranges[scan_.cluster_edges[i].second] * 
                                cos((scan_heading_cart + scan_.angles[scan_.cluster_edges[i].second] - 90)* PI/180);
        
        const float g_right_bound_y = scan_.scan_pose.y - scan_.ranges[scan_.cluster_edges[i].second] * 
                                sin((scan_heading_cart + scan_.angles[scan_.cluster_edges[i].second] - 90)* PI/180);

        const float g_left_bound_x = scan_.scan_pose.x + scan_.ranges[scan_.cluster_edges[i+1].first] * 
                                cos((scan_heading_cart + scan_.angles[scan_.cluster_edges[i+1].first] - 90)* PI/180);

        const float g_left_bound_y = scan_.scan_pose.y - scan_.ranges[scan_.cluster_edges[i+1].first] * 
                                sin((scan_heading_cart + scan_.angles[scan_.cluster_edges[i+1].first] - 90)* PI/180);

        scan_.gateway.push_back({{g_right_bound_x, g_right_bound_y},{g_left_bound_x, g_left_bound_y}});
        scan_.gateway_blocking.push_back({{g_right_bound_x, g_right_bound_y},{g_left_bound_x, g_left_bound_y}});

        //std::cout<<"{("<<g_right_bound_x<<","<<g_right_bound_y<<"),("<<g_left_bound_x<<","<<g_left_bound_y<<")},";
    }

    //std::cout<<"\n6";
    return scan_.gateway;
}

//check if there's a direct path (with obstacle clearance) toward the goal within the current scan region

bool LocalAnalysis::ScanAnalysis::directGoalPath(LidarScan& scan_, Vector3d goal_pose_){

    //convert from compass heading to cartesian heading
    float scan_heading_cart = 90 - scan_.scan_pose.z;
    scan_heading_cart = (scan_heading_cart < 0 ? scan_heading_cart + 360 : scan_heading_cart);
    //determine global goal heading from current scan pose
    const float g_goal_heading = (atan2(scan_.scan_pose.y - goal_pose_.y, goal_pose_.x - scan_.scan_pose.x))*180/PI;
    
    //determine local goal heading from current scan pose
    float l_goal_heading = (g_goal_heading - scan_heading_cart + 90);

    l_goal_heading = l_goal_heading < 0 ? l_goal_heading + 360 : l_goal_heading; 
    l_goal_heading = l_goal_heading >= 360 ? l_goal_heading - 360: l_goal_heading;

    //determine local goal distance from current scan pose
    const float l_goal_distance = sqrt(pow( goal_pose_.x - scan_.scan_pose.x, 2) 
                                + pow(scan_.scan_pose.y - goal_pose_.y, 2));

    /*starting at the local map's origin, steps and searches toward the goal at a given step size. Within each step, 
     check for obstacle interference using the distances between local scan data and current step position */
    //std::cout<<"\nl_goal_heading:"<<l_goal_heading<<" l_goal_distance:"<< l_goal_distance;
    if(l_goal_distance <= scan_.detection_range && l_goal_heading <= 180){
        //std::cout<<"\nchecking";
        const float SEARCH_STEP_SIZE = 0.5;
        float search_step = 0;
        bool end = false;
        while(search_step * SEARCH_STEP_SIZE <= l_goal_distance){
            //current search position
            const float search_x = search_step * SEARCH_STEP_SIZE * cos(l_goal_heading * PI / 180);
            const float search_y = search_step * SEARCH_STEP_SIZE * sin(l_goal_heading * PI / 180);
            //scan data positions   
            //std::cout<<"\ngoal path check:";
            for(int i = 0; i < scan_.num_data; i++){
                const float scan_data_x = scan_.ranges[i] * cos(scan_.angles[i] * PI / 180);
                const float scan_data_y = scan_.ranges[i] * sin(scan_.angles[i] * PI / 180);
                const float distance = sqrt(pow(search_x - scan_data_x, 2) + pow(search_y - scan_data_y, 2));
                //std::cout<<"("<<scan_data_x<<","<<scan_data_y<<","<<distance<<")";
                if(distance < scan_.grouping_radius/3){
                    
                    return false;
                }
            }
            
            search_step++;
            if(end){
                
                return true;
            }
            
            if(search_step * SEARCH_STEP_SIZE > l_goal_distance){
                
                end = true;
                search_step = l_goal_distance / SEARCH_STEP_SIZE;
            }
        }
        return true;
    }else{
        return false;
    }
}



//return the angle between two global directions
float LocalAnalysis::LidarSim::angleBetween(float dir_1, float dir_2){
    dir_1 = (dir_1 < 0 ? dir_1 + 360 : dir_1);
    dir_1 = (dir_1 > 360 ? dir_1 - 360 : dir_1);
    dir_2 = (dir_2 < 0 ? dir_2 + 360 : dir_2);
    dir_2 = (dir_2 > 360 ? dir_2 - 360 : dir_2);
    float diff = abs(dir_1 - dir_2);
    if(diff <= 180){
        return diff;
    }else{
        return 360 - diff;
    }
}

//Check if a cell is within the scan's angular range, min_scan_angle is the global cartesian heading 
//of the minimum angle (0 degree) of the scan
bool LocalAnalysis::LidarSim::inAngularScanRange(float min_scan_angle, float cell_dir){
    if(min_scan_angle <= 180){
        return cell_dir >= min_scan_angle && cell_dir <= min_scan_angle + 180;
    }else{
        return cell_dir >= min_scan_angle || cell_dir <= int(min_scan_angle + 180) % 360;
    }
}

//return the distance between two scan data
float LocalAnalysis::ScanAnalysis::ScanDataDistance(LidarScan &scan_, int index_1_, int index_2_){
    return sqrt(pow(scan_.ranges[index_1_], 2) + pow(scan_.ranges[index_2_], 2) 
    - 2 * scan_.ranges[index_1_] * scan_.ranges[index_2_] 
    * cos(abs(scan_.angles[index_1_] - scan_.angles[index_2_]) * PI / 180));
}





