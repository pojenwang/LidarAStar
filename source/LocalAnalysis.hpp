#pragma once

#include "LidarAStar.hpp"

using namespace std;
using namespace LidarAStar;

namespace LocalAnalysis{

    struct LidarScan{
        Vector3d                                        scan_pose;       //{x,y,h}
        vector<float>                                   angles;
        vector<float>                                   ranges;
        vector<int>                                     labels;
        vector<pair<int,int>>                           cluster_edges;   // pair<right_edge, left_edge>, stores INDEXES of cluster edges
        vector<pair<Vector2d,Vector2d>>                 gateway;         // gateways for traversing
        vector<pair<Vector2d,Vector2d>>                 gateway_blocking;// gateways for drawing imaginary boundaries on map
        int                                             num_cluster;
        int                                             num_data;
        float                                           grouping_radius;
        float                                           detection_range;
        bool                                            untraversable_gateway_error;
        bool                                            no_obstacle_error;
    };

    class LidarSim{

    public:
        static LidarScan    simulateLidarScan(cv::Mat map, Vector3d scan_pose_, float range_, int num_data_);
        static LidarScan    simulateLidarScan2(cv::Mat map, Vector3d scan_pose_, float range_, int num_data_);
        static void         printLidarScan(LocalAnalysis::LidarScan scan_);
        static float        angleBetween(float dir_1, float dir_2);
        static bool         inAngularScanRange(float min_scan_angle, float cell_dir);
    };

    class ScanAnalysis{

    public:
        static vector<pair<Vector2d,Vector2d>> 
                            getGatewaysFromScan(LidarScan& scan_, pair<Vector2d,Vector2d> edge_pts,
                                                float grouping_radius_, float valid_range_);
        static float        ScanDataDistance(LidarScan& scan_, int index_1_, int index_2_);
        static bool         directGoalPath(LidarScan& scan_, Vector3d goal_pose_);
        static void         getGatwayScanPose();
    };
}