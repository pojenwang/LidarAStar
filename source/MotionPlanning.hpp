#pragma once
#include "LidarAStar.hpp"
#include "LocalAnalysis.hpp"

using namespace LidarAStar;

namespace MotionPlan{
    struct Polar{
        float dir;
        float dis;
    };

    struct Vehicle{
        Vector3d                            start_pose;
        Vector3d                            goal_pose;
        vector<LidarAStar::Gateway>         gateway_path;
        Vector3d                            curr_pose;
        float                               velocity;
        float                               acceleration;
        float                               target_steer_angle;
        float                               steer_angle;
        float                               steer_omega;
        float                               turn_radius;
        float                               max_velocity;
        float                               max_acceleration;   
        float                               max_steer_angle; 
        float                               max_steer_omega;
        float                               err_cross_track;
        float                               err_steer_angle;
        float                               prediction_time;
        float                               collision_check_time;
        float                               dt;
        LocalAnalysis::LidarScan            contour;
        float                               width;
        float                               length;
        bool                                constraint;
        vector<Vector3d>                    lidar_astar_path;
        vector<LocalAnalysis::LidarScan>    global_contour_path;
        
        public:
        void                            getLidarAStarPath(Vector3d start_pose_, Vector3d goal_pose_, 
                                                pair<vector<LidarAStar::Gateway>, bool> gateway_path_, bool constraint_);
        void                            getVelocityProfile();
        void                            printLidarAStarPathOnMap(cv::Mat& map_);
        float                           fix_direction_360(float direction_);
        
        private:
        void                            initializeVehicleState();
        void                            setGatewayPath(vector<LidarAStar::Gateway> gateway_path_, Vector3d start_pose_, 
                                                        Vector3d goal_pose_, bool constraint_);
        void                            setVehicelContour(LocalAnalysis::LidarScan contour_scan_, float width_, float length_);
        pair<Polar, Polar>              getLocalGatewayPolarCoor(LidarAStar::Gateway gateway_);
        pair<pair<Polar, Polar>, float> getLocalSteerLimits(pair<Polar, Polar> local_gateway_coor);
        float                           getTargetSteerAngle(pair<pair<Polar, Polar>, float> obs_steer_limits_curr_, 
                                                            pair<pair<Polar, Polar>, float> obs_steer_limits_next_,
                                                            bool last_gateway_);                                      
        void                            updateState();
        Polar                           getGoalSteerAngleDistance();
        bool                            poseCollisionCheck(Vector3d pose_);
        
    };

}