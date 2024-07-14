#include "LidarAStar.hpp"
#include "LocalAnalysis.hpp"

//change this to true to print search details on the terminal
#define print_gateway_search    true

pair<vector<LidarAStar::Gateway>, bool> LidarAStar::AStar::findGatewayPath(cv::Mat& map_rgb, Vector3d start_pose_, Vector3d goal_pose_){    

    cv::Mat map_gray = map_rgb.clone();
    cv::cvtColor(map_rgb,map_gray,cv::COLOR_RGB2GRAY);

    LidarAStar::Gateway start = Gateway();
    initStartGateway(start, start_pose_, goal_pose_);

    vector<LidarAStar::Gateway> gateways;  
    gateways.reserve(1000);
    gateways.push_back(start);

    Gateway *current = nullptr;
    bool goal_path_at_start = false;
    int iter = 1;
    int max_iter = 30;
    while(!gateways.empty() && iter <= max_iter && cv::waitKey()){
        current = findMinCostGateway(gateways);
        current->is_open = false;
        Vector3d curr_scan_pose = current->scan_pose;

        if(iter > 1){
            printGatewayToMap(*current, map_rgb, 0, cv::Scalar(255,150,10),2);
            printGatewayEdgeToMap(*current, *current->parent, map_rgb, cv::Scalar(0,0,255),2);
        }

        const int center_x = current->scan_pose.x * PIX_PER_METER;
        const int center_y = current->scan_pose.y * PIX_PER_METER;
        circle(map_rgb, cv::Point(center_x, center_y), 8, cv::Scalar(0,0,255), 2, 1, 0);

        if(print_gateway_search){
            std::cout<<"\n\n-----------------------------------iter:"<<iter<<"--------------------------------------------\n";
            std::cout<<"current scan pos:("<<current->scan_pose.x<<", "<<current->scan_pose.y<<", "<<current->scan_pose.z
                <<"), par:(";
            if(iter != 1){
                std::cout<<current->parent->scan_pose.x<<","<<current->parent->scan_pose.y<<","<<current->parent->scan_pose.z<<")\n";

            }
        }

        LocalAnalysis::LidarScan sim_scan = LocalAnalysis::LidarSim::simulateLidarScan2(map_gray, current->scan_pose, 16, 181); 
        

        /*
        if(LocalAnalysis::ScanAnalysis::directGoalPath(sim_scan, goal_pose_)){
            if(iter == 1){
                goal_path_at_start = true;
                std::cout<<"\ndirect goal path at start!";
            }
            break;
        } */ 
        current->neighboring_gateways = LocalAnalysis::ScanAnalysis::getGatewaysFromScan(sim_scan,current->edge_pts, 5, 16);  
        if(print_gateway_search)
        //  LocalAnalysis::LidarSim::printLidarScan(sim_scan);

        if(current->neighboring_gateways.size() == 0){
            iter++;
            cv::imshow("map", map_rgb);
            continue;
        }

        if(print_gateway_search)
            std::cout<<"\nneighboring gateways:"<<current->neighboring_gateways.size()<<"\n";
        
        for(auto gateway_edge_pts:current->neighboring_gateways){
            Gateway neighbor = Gateway();             
            neighbor.parent = findGatewayFromList(gateways, curr_scan_pose);
            neighbor.edge_pts = gateway_edge_pts;
            neighbor.length = getGatewayLength(neighbor);
            neighbor.scan_pose = getGatwayScanPose(neighbor, 0.0);
            neighbor.g_cost = getGatewayG_Cost(*current, neighbor); 
            neighbor.h_cost = 0;//getGatewayH_Cost(neighbor, goal_pose_);
            neighbor.is_open = true;  

            if(print_gateway_search){
                std::cout<<"\n\nscan pos:("<<neighbor.scan_pose.x<<","<<neighbor.scan_pose.y<<","<<neighbor.scan_pose.z<<")";         
                std::cout<<"\ng_cost:"<<neighbor.g_cost;
                std::cout<<" h_cost:"<<neighbor.h_cost;
            }

            printGatewayToMap(neighbor, map_rgb, 0, cv::Scalar(0, 255, 0), 2);
            printGatewayToMap(neighbor, map_gray, 0.2, cv::Scalar(255), 1); 
            gateways.push_back(neighbor);               

        }
        iter++;
        removeFromList(gateways, current);

        cv::imshow("map", map_rgb);

        if(print_gateway_search){
            if(iter != 1){
                std::cout<<"\n\nOpen gateway list:"<<gateways.size();
                for(auto gateway : gateways){
                    if(gateway.is_open){
                        
                        std::cout<<"\n{("<<gateway.scan_pose.x<<","<<gateway.scan_pose.y<<","
                        <<gateway.scan_pose.z<<"), costs:"<<gateway.g_cost+gateway.h_cost<<"}, parent:("
                        <<gateway.parent->scan_pose.x<<","<<gateway.parent->scan_pose.y<<","
                        <<gateway.parent->scan_pose.z<<")";
                    }
                }   
            }
        }         
    }

    vector<LidarAStar::Gateway> gateway_path;
    max_iter = iter;
    iter = 1;
    while(current != nullptr && iter <= max_iter){
        gateway_path.push_back(*current);
        //printGatewayToMap(*current, map_rgb, 0, cv::Scalar(18,200,0),1);
        current = current->parent;

        iter++;
    }
    reverse(gateway_path.begin(),gateway_path.end());
    gateway_path.erase(gateway_path.begin());

    if(gateway_path.size() == 0 && !goal_path_at_start){
        std::cout<<"No gateway path found!";
    }
    /*
    std::cout<<"\n\nfinal_gateway:";
    for(auto gateway:gateway_path){
        std::cout<<"\n{("<<gateway.edge_pts.first.x()<<","<<gateway.edge_pts.first.y()<<"),("
        <<gateway.edge_pts.second.x()<<","<<gateway.edge_pts.second.y()<<")}";
    }*/

    return {gateway_path, goal_path_at_start};
}

bool LidarAStar::operator==(Vector2d &v1, Vector2d &v2){
    return v1.x == v2.x && v1.y == v2.y;
}

bool LidarAStar::operator==(Vector3d &v1, Vector3d &v2){
    return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}
/*
void LidarAStar::Vector2d(double x_, double y_){
    x = x_;
    y = y_;
}

void LidarAStar::Vector3d(double x_, double y_, double z_){
    x = x_;
    y = y_;
    z = z_;
}*/

void LidarAStar::AStar::initStartGateway(LidarAStar::Gateway& start_gateway_, Vector3d start_pose_, Vector3d goal_pose_){
    start_gateway_.parent = nullptr;
    start_gateway_.scan_pose = start_pose_;
    start_gateway_.is_open = true;
    start_gateway_.g_cost = 0;
    start_gateway_.h_cost = 0; //LidarAStar::AStar::getGatewayH_Cost(start_gateway_, goal_pose_);
    float right_dummy_x = start_pose_.x + 0.5 * cos(-start_pose_.z * PI / 180);
    float right_dummy_y = start_pose_.y - 0.5 * sin(-start_pose_.z * PI / 180);
    float left_dummy_x = start_pose_.x + 0.5 * cos(180 - start_pose_.z * PI / 180);
    float left_dummy_y = start_pose_.y - 0.5 * sin(180 - start_pose_.z * PI / 180);
    start_gateway_.edge_pts = {{right_dummy_x, right_dummy_y},{left_dummy_x, left_dummy_y}};
    start_gateway_.length = 1;
}

LidarAStar::Gateway* LidarAStar::AStar::findMinCostGateway(vector<LidarAStar::Gateway>& gateway_list_){
    LidarAStar::Gateway *ptr = &gateway_list_[0];
    float min_cost = INT_MAX;
    for(uint i =0; i < gateway_list_.size(); i++){
        if(gateway_list_[i].is_open && gateway_list_[i].g_cost + gateway_list_[i].h_cost < min_cost){
            ptr = &gateway_list_[i];
            min_cost = gateway_list_[i].g_cost + gateway_list_[i].h_cost;
        }
    }
    return ptr;
}

LidarAStar::Gateway* LidarAStar::AStar::findGatewayFromList(vector<LidarAStar::Gateway>& gateway_list_, Vector3d& scan_pose_){
    LidarAStar::Gateway *ptr = &gateway_list_[0];
    
    for(uint i =0; i < gateway_list_.size(); i++){
        if(gateway_list_[i].scan_pose == scan_pose_){
            ptr = &gateway_list_[i];
            return ptr;
        }
    }

    return nullptr;
}

void LidarAStar::AStar::insertToList(vector<LidarAStar::Gateway>& gateway_list_, LidarAStar::Gateway& new_gateway_){     
    bool inserted = false;
    for(int i = gateway_list_.size()-1; i >=0 ; --i){
        if(new_gateway_.g_cost + new_gateway_.h_cost < gateway_list_[i].g_cost + gateway_list_[i].h_cost){
            gateway_list_.insert(gateway_list_.begin() + i + 1, new_gateway_);
            inserted = true;
            break;
        }
    }
    if(!inserted)
        gateway_list_.insert(gateway_list_.begin(), new_gateway_);

}

void LidarAStar::AStar::removeFromList(vector<LidarAStar::Gateway>& gateway_list_, LidarAStar::Gateway* old_gateway_){
    for(int i = gateway_list_.size()-1; i >=0 ; --i){
        if(gateway_list_[i].scan_pose == old_gateway_->scan_pose){
            //gateway_list_.erase(gateway_list_.begin() + i);
            //gateway_list_[i].scan_pose = {0,0,0};
            gateway_list_[i].g_cost = INT_MAX;
        }
    }
}

LidarAStar::Vector3d LidarAStar::AStar::getGatwayScanPose(LidarAStar::Gateway& gateway_, float shift_dist_){
    const float scan_pose_x = (gateway_.edge_pts.first.x + gateway_.edge_pts.second.x)/2;
    const float scan_pose_y = (gateway_.edge_pts.first.y + gateway_.edge_pts.second.y)/2;
    const float dy = gateway_.edge_pts.second.y - gateway_.edge_pts.first.y;
    const float dx = gateway_.edge_pts.first.x - gateway_.edge_pts.second.x;
    float gateway_heading = -atan2(dy, dx) * 180 / PI;
    gateway_heading = (gateway_heading < 0 ? gateway_heading + 360: gateway_heading);
    const float shifted_scan_pose_x = scan_pose_x + shift_dist_ * cos((90-gateway_heading) * PI/180);
    const float shifted_scan_pose_y = scan_pose_y + shift_dist_ * sin((90-gateway_heading) * PI/180);
    
    //std::cout<<"\n\nGatwayScanPose:("<<scan_pose_x<<","<<scan_pose_y<<","<<gateway_heading<<")";
    LidarAStar::Vector3d result;
    result.x = shifted_scan_pose_x;
    result.y = shifted_scan_pose_y;
    result.z = gateway_heading;
    return result;
}

float LidarAStar::AStar::getGatewayLength(LidarAStar::Gateway& gateway_){
    return sqrt(pow(gateway_.edge_pts.first.x - gateway_.edge_pts.second.x,2) 
                                    + pow(gateway_.edge_pts.first.y - gateway_.edge_pts.second.y,2));
}

float LidarAStar::AStar::getGatewayG_Cost(LidarAStar::Gateway& gateway_from_, LidarAStar::Gateway& gateway_to_){
    return gateway_from_.g_cost + sqrt(pow(gateway_to_.scan_pose.x - gateway_from_.scan_pose.x,2) + 
        pow(gateway_to_.scan_pose.y - gateway_from_.scan_pose.y,2)); 
}

float LidarAStar::AStar::getGatewayH_Cost(LidarAStar::Gateway& gateway_, LidarAStar::Vector3d goal_pose_){
    return sqrt(pow(gateway_.scan_pose.x - goal_pose_.x,2) + 
        pow(gateway_.scan_pose.y - goal_pose_.y,2));
}

void LidarAStar::AStar::printGatewayToMap(LidarAStar::Gateway& gateway_, cv::Mat map_, float shift_dist_, 
                                            cv::Scalar color_, int thickness_){
    
    float dx = shift_dist_ * cos((270-gateway_.scan_pose.z) * PI/180);
    float dy = shift_dist_ * sin((270-gateway_.scan_pose.z) * PI/180);
    int x_1 = (gateway_.edge_pts.first.x + dx) * PIX_PER_METER;
    int y_1 = (gateway_.edge_pts.first.y - dy) * PIX_PER_METER;
    int x_2 = (gateway_.edge_pts.second.x + dx ) * PIX_PER_METER;
    int y_2 = (gateway_.edge_pts.second.y - dy ) * PIX_PER_METER;
    x_1 = min(x_1, map_.cols-1);
    x_1 = max(0, x_1);
    y_1 = min(y_1, map_.rows-1);
    y_1 = max(0, y_1);
    x_2 = min(x_2, map_.cols-1);
    x_2 = max(0, x_2);
    y_2 = min(y_2, map_.rows-1);
    y_2 = max(0, y_2); 

    if(thickness_ == 2){
        //std::cout<<"final path:("<<x_1<<","<<y_1<<"),("<<x_2<<","<<y_2<<")\n";
    }
    cv::line(map_, cv::Point(x_1,y_1), cv::Point(x_2,y_2), color_, thickness_, 8);
}

void LidarAStar::AStar::printGatewayEdgeToMap(LidarAStar::Gateway& gateway1_, LidarAStar::Gateway& gateway2_, 
    cv::Mat map_, cv::Scalar color_, int thickness_){
    
    int x_1 = gateway1_.scan_pose.x * PIX_PER_METER;
    int y_1 = gateway1_.scan_pose.y * PIX_PER_METER;
    int x_2 = gateway2_.scan_pose.x * PIX_PER_METER;
    int y_2 = gateway2_.scan_pose.y * PIX_PER_METER;
    x_1 = min(x_1, map_.cols-1);
    x_1 = max(0, x_1);
    y_1 = min(y_1, map_.rows-1);
    y_1 = max(0, y_1);
    x_2 = min(x_2, map_.cols-1);
    x_2 = max(0, x_2);
    y_2 = min(y_2, map_.rows-1);
    y_2 = max(0, y_2); 

    if(thickness_ == 2){
        //std::cout<<"final path:("<<x_1<<","<<y_1<<"),("<<x_2<<","<<y_2<<")\n";
    }
    cv::line(map_, cv::Point(x_1,y_1), cv::Point(x_2,y_2), color_, thickness_, 8);
}