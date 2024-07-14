#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include <queue>
#include <list>
#include <string>
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"


#define   PI                3.14159
#define   PIX_PER_METER     10
using namespace std;


namespace LidarAStar{
    struct Vector2d{
        double x;
        double y;        
    };
    bool   operator==(Vector2d &v1, Vector2d &v2);

    struct Vector3d{
        double x;
        double y;
        double z;   
    };
    bool   operator==(Vector3d &v1, Vector3d &v2);

    struct Gateway{
        float                           g_cost;
        float                           h_cost; 
        pair<Vector2d,Vector2d>         edge_pts;
        float                           length;
        Vector3d                        scan_pose;
        vector<pair<Vector2d,Vector2d>> neighboring_gateways;
        Gateway                         *parent;
        bool                            is_open;   
    };

    class AStar{
        
    public:
        static void             initStartGateway(LidarAStar::Gateway& start_gateway_, Vector3d start_pose_, Vector3d goal_pose_);
        static pair<vector<Gateway>, bool>  findGatewayPath(cv::Mat& map, Vector3d start_pose_, Vector3d goal_pose_);
        static void             insertToList(vector<LidarAStar::Gateway>& gateway_list_, LidarAStar::Gateway& new_gateway_);
        static void             removeFromList(vector<LidarAStar::Gateway>& gateway_list_, LidarAStar::Gateway* old_gateway_);
        static float            getGatewayLength(LidarAStar::Gateway& gateway_);
        static float            getGatewayG_Cost(Gateway& gateway_from_, Gateway& gateway_to_);
        static float            getGatewayH_Cost(Gateway& gateway_, Vector3d goal_pose_);
        static Gateway*         findGatewayFromList(vector<LidarAStar::Gateway>& gateway_list_, Vector3d& scan_pose_);
        static Vector3d         getGatwayScanPose(LidarAStar::Gateway& gateway_, float shift_dist_);//Vector2d right_bound_, Vector2d left_bound_);
        static void             printGatewayToMap(LidarAStar::Gateway& gateway_, cv::Mat map_, float shift_dist_, 
                                                    cv::Scalar color_, int thickness_);
        static void             printGatewayEdgeToMap(LidarAStar::Gateway& gateway1_, LidarAStar::Gateway& gateway2_, 
                                                    cv::Mat map_, cv::Scalar color_, int thickness_);
        static Gateway*         findMinCostGateway(vector<LidarAStar::Gateway>& gateway_list_);
    };

}