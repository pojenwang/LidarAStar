#include "MotionPlanning.hpp"
#include "LidarAStar.hpp"

using namespace MotionPlan;

//change this to true to print motion details on the terminal
#define print_motion       false

void Vehicle::initializeVehicleState(){
    velocity = 0;               // m/s
    acceleration = 0;           // m/s^2
    steer_angle = 0;            // deg
    steer_omega = 0;            // deg/sec
    turn_radius = 0;            // m
    max_velocity = 0;           // m/s
    max_acceleration = 0;       // m/s^2
    max_steer_angle = 50;       // deg
    max_steer_omega = 40;        // deg/sec
    err_cross_track = 0;        // m
    err_steer_angle = 0;        // deg
    prediction_time = 100;        // sec
    collision_check_time = 10;   // sec
    dt = 0.05;                  // sec
}

void Vehicle::setGatewayPath(vector<LidarAStar::Gateway> gateway_path_, Vector3d start_pose_, 
                                        Vector3d goal_pose_, bool constraint_){
    gateway_path = gateway_path_;
    start_pose = start_pose_;
    goal_pose = goal_pose_;
    curr_pose = start_pose;
    constraint = constraint_;
}

void Vehicle::setVehicelContour(LocalAnalysis::LidarScan contour_scan_, float width_, float length_){
    contour = contour_scan_;
    width = width_;
    length = length_;
}

pair<Polar, Polar> Vehicle::getLocalGatewayPolarCoor(LidarAStar::Gateway gateway_){
    //convert from compass heading to cartesian heading
    float heading_cart = 90 - curr_pose.z;
    heading_cart = (heading_cart < 0 ? heading_cart + 360 : heading_cart);

    if(print_motion){
        std::cout<<"\n\nobs_glo_xy:("<<gateway_.edge_pts.first.x<<","<<gateway_.edge_pts.first.y<<")";
        std::cout<<"("<<gateway_.edge_pts.second.x<<","<<gateway_.edge_pts.second.y<<")";
    }

    const float g_right_edge_heading = atan2(curr_pose.y - gateway_.edge_pts.first.y, 
                                        gateway_.edge_pts.first.x - curr_pose.x) * 180/PI;

    const float g_left_edge_heading = atan2(curr_pose.y - gateway_.edge_pts.second.y, 
                                        gateway_.edge_pts.second.x - curr_pose.x) * 180/PI;  
    
    Polar right_edge, left_edge;
    //determine local goal heading from current vehicle pose
    right_edge.dir = (g_right_edge_heading - heading_cart + 90);
    right_edge.dir = right_edge.dir < 0 ? right_edge.dir + 360 : right_edge.dir; 
    right_edge.dir = right_edge.dir >= 360 ? right_edge.dir - 360: right_edge.dir;

    left_edge.dir = (g_left_edge_heading - heading_cart + 90);
    left_edge.dir = left_edge.dir < 0 ? left_edge.dir + 360 : left_edge.dir; 
    left_edge.dir = left_edge.dir >= 360 ? left_edge.dir - 360: left_edge.dir;

    //determine local goal distance from current vehicle pose
    right_edge.dis = sqrt(pow( curr_pose.x - gateway_.edge_pts.first.x, 2) 
                                + pow(gateway_.edge_pts.first.y - curr_pose.y, 2));

    left_edge.dis = sqrt(pow( curr_pose.x - gateway_.edge_pts.second.x, 2) 
                                + pow(gateway_.edge_pts.second.y - curr_pose.y, 2));            
    //std::cout<<"\nobs_lo_polar:{{"<<right_edge.dir<<","<<right_edge.dis<<"},{"<<left_edge.dir<<","<<left_edge.dis<<"}}";
    return {right_edge, left_edge};
}

pair<pair<Polar, Polar>, float> Vehicle::getLocalSteerLimits(pair<Polar, Polar> local_gateway_){
    const float l_right_edge_x = local_gateway_.first.dis * cos(local_gateway_.first.dir *  PI/180);
    const float l_right_edge_y = local_gateway_.first.dis * sin(local_gateway_.first.dir * PI/180);
    const float l_left_edge_x = local_gateway_.second.dis * cos(local_gateway_.second.dir * PI/180);
    const float l_left_edge_y = local_gateway_.second.dis * sin(local_gateway_.second.dir * PI/180);
    const float gateway_length = sqrt(pow(l_right_edge_x - l_left_edge_x, 2) + pow(l_right_edge_y - l_left_edge_y, 2));
    const float l_mid_pt_x = (l_right_edge_x + l_left_edge_x)/2;
    const float l_mid_pt_y = (l_right_edge_y + l_left_edge_y)/2;

    if(print_motion){
        std::cout<<"\ngateway_length:"<<gateway_length;
        std::cout<<"\nlocal edges_xy:{{"<<l_right_edge_x<<","<<l_right_edge_y<<"},{"<<l_left_edge_x<<","<<l_left_edge_y<<"}}";
        std::cout<<"local_gateway_dir:{"<<local_gateway_.first.dir<<","<<local_gateway_.second.dir<<"}";
    }
    
    bool add_clearance_right = local_gateway_.first.dir > 30 && local_gateway_.first.dir < 150 && gateway_length > 2.5;
    bool add_clearance_left = local_gateway_.second.dir > 30 && local_gateway_.second.dir < 150 && gateway_length > 2.5;

    float extra_clearance_right =  add_clearance_right ? 0.6 : 0;
    float extra_clearance_left = add_clearance_left ? 0.6 : 0;

    width = 0.4;
    Polar right_edge, left_edge;
    right_edge.dir = atan2(l_right_edge_y - length * (1), l_right_edge_x - (width + extra_clearance_right))*180/PI;
    left_edge.dir = atan2(l_left_edge_y - length * (1), l_left_edge_x + (width + extra_clearance_left))*180/PI;
    float mid_dir = atan2(l_mid_pt_y,l_mid_pt_x)*180/PI;
       
    right_edge.dir = fix_direction_360(right_edge.dir);
    right_edge.dir = (right_edge.dir > 180 && right_edge.dir <= 270) ? 180 : right_edge.dir;
    right_edge.dir = (right_edge.dir > 270 && right_edge.dir <= 360) ? 0 : right_edge.dir;
    left_edge.dir = fix_direction_360(left_edge.dir); 
    left_edge.dir = (left_edge.dir > 180 && left_edge.dir <= 270) ? 180 : left_edge.dir;
    left_edge.dir = (left_edge.dir > 270 && left_edge.dir <= 360) ? 0 : left_edge.dir;

    right_edge.dis = sqrt(pow(l_right_edge_y - length * (1), 2) + pow(l_right_edge_x - width * (2), 2));
    left_edge.dis = sqrt(pow(l_left_edge_y - length * (1), 2) + pow(l_left_edge_x - width * (2), 2));
    
    if(print_motion){
        std::cout<<"\nsteer_polar:{{"<<right_edge.dir<<","<<right_edge.dis<<"},{"<<left_edge.dir<<","<<left_edge.dis
        <<"},"<<mid_dir<<"}";
    }

    return {{right_edge, left_edge}, mid_dir};
}

float Vehicle::getTargetSteerAngle(pair<pair<Polar, Polar>, float> obs_steer_limits_curr_, 
                                            pair<pair<Polar, Polar>, float> obs_steer_limits_next_,
                                            bool last_gateway_){
    
    const float MID_THRESHOLD = 70;
    
    float target_steer_angle;
    const float curr_right_bound_dir = obs_steer_limits_curr_.first.first.dir;
    const float curr_right_bound_dis = obs_steer_limits_curr_.first.first.dis;
    const float curr_left_bound_dir = obs_steer_limits_curr_.first.second.dir;
    const float curr_left_bound_dis = obs_steer_limits_curr_.first.second.dis;
    const float curr_mid_dir = obs_steer_limits_curr_.second;

    const float next_right_bound_dir = obs_steer_limits_next_.first.first.dir;
    const float next_right_bound_dis = obs_steer_limits_next_.first.first.dis;
    const float next_left_bound_dir = obs_steer_limits_next_.first.second.dir;
    const float next_left_bound_dis = obs_steer_limits_next_.first.second.dis;
    const float next_mid_dir = !last_gateway_? obs_steer_limits_next_.second : 90 - getGoalSteerAngleDistance().dir; 

    float next_gateway_dir;
    if(next_right_bound_dir > next_left_bound_dir){
        next_gateway_dir = next_right_bound_dis < next_left_bound_dis ? next_right_bound_dir : next_left_bound_dir;
    }else{
        next_gateway_dir = next_mid_dir;
    }

    if(print_motion)
        std::cout<<"\nnext_gateway_dir"<<next_gateway_dir;

    //std::cout<<"\nnext_mid_dir:"<<next_mid_dir;
    //if(last_gateway_){
    //    std::cout<<"\ngoal dir:"<<getGoalSteerAngleDistance().dir;
    //}   
    //if the directions crosses over, return the direction with smaller distance

    if(curr_right_bound_dir > curr_left_bound_dir){
        //std::cout<<"\ncross over!";
        target_steer_angle = curr_right_bound_dis < curr_left_bound_dis ? curr_right_bound_dir : curr_left_bound_dir;
                                
    }else{        
        float angle_between = curr_left_bound_dir - curr_right_bound_dir;
        if(angle_between < MID_THRESHOLD){
            target_steer_angle = curr_mid_dir;
        }else{
            if(next_gateway_dir >= curr_right_bound_dir && 
                next_gateway_dir <= curr_left_bound_dir){
                target_steer_angle = next_gateway_dir;
            }else{
                target_steer_angle = next_gateway_dir < curr_right_bound_dir ? 
                                    curr_right_bound_dir : curr_left_bound_dir;
            }
        }
    }
    target_steer_angle = -(target_steer_angle - 90);
    target_steer_angle = target_steer_angle > 180 ? target_steer_angle - 360 : target_steer_angle;
    //std::cout<<"\nsteerAbgle:"<<target_steer_angle;
    return target_steer_angle;
}

void Vehicle::updateState(){
    /*
        vehicle model:
        x = u * cos(theta) * dt
        y = u * sin(theta) * dt
        theta = u / l * tan(psi) * dt
    */
    if(constraint){
        if(abs(target_steer_angle - steer_angle) < steer_omega * dt)
            steer_angle = target_steer_angle;
        if(steer_angle < target_steer_angle)
            steer_angle += max_steer_omega * dt;
        if(steer_angle > target_steer_angle)  
            steer_angle -= max_steer_omega * dt;
        steer_angle = min(steer_angle, max_steer_angle);
        steer_angle = max(steer_angle, -max_steer_angle);

    }else{
        steer_angle = target_steer_angle;
        steer_angle = min(steer_angle, max_steer_angle);
        steer_angle = max(steer_angle, -max_steer_angle);
    }
    length = 0.5;
    width = 0.5;
    float velocity_temp = 0.5; //this velocity is used to generate a path first, velocity profile will be determined next
    //kinematic model of a car like vehicle
    steer_angle = steer_angle == 0 ? 0.0001 : steer_angle;
    turn_radius = length/tan(steer_angle * PI/180);
    turn_radius = turn_radius > 10000 ? 10000 : turn_radius;
    turn_radius = turn_radius < -10000 ? -10000 : turn_radius;
    float omega = - velocity_temp / turn_radius * dt;

    if(print_motion){
        std::cout<<"\ntarget steer angle w/ constraints:"<<target_steer_angle;
        std::cout<<"\nbefore:("<<curr_pose.x<<","<<curr_pose.y<<","<<curr_pose.z<<")";
    }

    curr_pose.x += velocity_temp * cos((90 - curr_pose.z) * PI / 180) * dt;
    curr_pose.y -= velocity_temp * sin((90 - curr_pose.z) * PI / 180) * dt;
    curr_pose.z = fix_direction_360(curr_pose.z - omega * 180 / PI);

    if(print_motion){
        std::cout<<"\nafter:("<<curr_pose.x<<","<<curr_pose.y<<","<<curr_pose.z<<")"<<
        velocity_temp * cos((90 - curr_pose.z) * PI / 180) * dt<<" "<<
        velocity_temp * sin((90 - curr_pose.z) * PI / 180) * dt<<"\n";
    }
}

bool Vehicle::poseCollisionCheck(Vector3d pose_){
    return false;
}

void Vehicle::getLidarAStarPath(Vector3d start_pose_, Vector3d goal_pose_, 
                                pair<vector<LidarAStar::Gateway>,bool> gateway_path_ , bool constraint_){    
    initializeVehicleState();
    setGatewayPath(gateway_path_.first, start_pose_, goal_pose_, constraint_);
    uint scan_region_index = 0;
    if(gateway_path_.first.size() > 0){
        for(int i = 0; i <= prediction_time / dt; i++){
            bool last_gateway = (scan_region_index == gateway_path_.first.size()-1);
            if(scan_region_index < gateway_path.size()){
                pair<Polar, Polar> curr_gateway = getLocalGatewayPolarCoor(gateway_path[scan_region_index]);
                pair<Polar, Polar> next_gateway = last_gateway ? curr_gateway : 
                                                    getLocalGatewayPolarCoor(gateway_path[scan_region_index + 1]);
                pair<pair<Polar, Polar>, float> curr_steer_limits = getLocalSteerLimits(curr_gateway);            
                pair<pair<Polar, Polar>, float> next_steer_limits = getLocalSteerLimits(next_gateway);
                target_steer_angle = getTargetSteerAngle(curr_steer_limits, next_steer_limits, last_gateway);
                float angle_between = LocalAnalysis::LidarSim::angleBetween(curr_gateway.second.dir, curr_gateway.first.dir);        
                if(angle_between > 170)
                    scan_region_index++;
            }else{
                target_steer_angle = getGoalSteerAngleDistance().dir;
                if(getGoalSteerAngleDistance().dis < 0.2){
                    std::cout<<"\nGOAL REACHED!!!!";
                    break;
                }
            }
            lidar_astar_path.push_back(curr_pose);
            updateState();

            //std::cout<<"\nangle BT:"<<angle_between;
            //if(getGoalSteerAngleDistance().dis< 0.1)
            //    break;
        }
    }else{
        //There's a direct goal path at the start pose (no gateway)
        if(gateway_path_.second){
            for(int i = 0; i <= prediction_time / dt; i++){
                target_steer_angle = getGoalSteerAngleDistance().dir;
                if(getGoalSteerAngleDistance().dis < 0.2){
                    std::cout<<"\nGOAL REACHED!!!!";
                    break;
                }
                lidar_astar_path.push_back(curr_pose);
                updateState();
            }
        }
    }
}

void Vehicle::getVelocityProfile(){

}

Polar Vehicle::getGoalSteerAngleDistance(){
    //const float curr_cart_heading = 90 - curr_pose.z();
    const float g_goal_heading = atan2(curr_pose.y - goal_pose.y, goal_pose.x - curr_pose.x) * 180 / PI;
    
    //determine local goal heading from current scan pose
    float l_goal_heading = g_goal_heading - (90 - curr_pose.z) + 90;
    l_goal_heading = l_goal_heading < 0 ? l_goal_heading + 360 : l_goal_heading; 
    l_goal_heading = l_goal_heading >= 360 ? l_goal_heading - 360: l_goal_heading;
    float goal_steer_angle = -(l_goal_heading - 90);

    const float goal_distance = sqrt(pow(goal_pose.y - curr_pose.y,2) + pow(goal_pose.x- curr_pose.x,2));
    //std::cout<<"\ncurr_pose:("<<curr_pose.x()<<","<<curr_pose.y()<<") "<< g_goal_heading<<" "<<goal_steer_angle;

    Polar goal;
    goal.dir = goal_steer_angle;
    goal.dis = goal_distance;
    return goal;
}

void Vehicle::printLidarAStarPathOnMap(cv::Mat& map_){  
    if(lidar_astar_path.size()>0){

        for(uint i = 0; i < lidar_astar_path.size()-1; i++){
            int x_1 = lidar_astar_path[i].x * PIX_PER_METER;
            int y_1 = lidar_astar_path[i].y * PIX_PER_METER;
            int x_2 = lidar_astar_path[i+1].x * PIX_PER_METER;
            int y_2 = lidar_astar_path[i+1].y * PIX_PER_METER;  
            x_1 = min(x_1, map_.cols-1);
            x_1 = max(0, x_1);
            y_1 = min(y_1, map_.rows-1);
            y_1 = max(0, y_1);
            x_2 = min(x_2, map_.cols-1);
            x_2 = max(0, x_2);
            y_2 = min(y_2, map_.rows-1);
            y_2 = max(0, y_2);   
            //std::cout<<"\n("<<x_1<<", "<<y_1<<")("<<x_2<<", "<<y_2<<")";
            cv::line(map_, cv::Point(x_1,y_1), cv::Point(x_2,y_2), cv::Scalar(50,50,250), 2, 8);
        }
    }
    cv::circle(map_, cv::Point(start_pose.x* PIX_PER_METER, start_pose.y* PIX_PER_METER), 
            0.5 * PIX_PER_METER, cv::Scalar(50,50,250), 1, 8, 0);
    cv::circle(map_, cv::Point(goal_pose.x* PIX_PER_METER, goal_pose.y* PIX_PER_METER), 
            0.5 * PIX_PER_METER, cv::Scalar(30,30,255), 1, 8, 0);           
}

float Vehicle::fix_direction_360(float direction_){
    while(direction_ < 0){
        direction_ += 360;
    }
    while(direction_ >= 360){
        direction_ -= 360;
    }
    return direction_;
}
