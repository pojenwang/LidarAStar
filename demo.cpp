#include <iostream>
#include <vector>
#include <memory>
#include <utility>
#include <chrono>
#include "source/LidarAStar.hpp"
#include "source/MotionPlanning.hpp"

using namespace std::chrono;

int main(int argc, char** argv) {

  Vector3d start_pose = {2, 28, 0};
  Vector3d goal_pose = {28, 2, 90};

  cv::Mat global_map_rgb = cv::imread("/home/popo/LidarAStar/map.jpg",cv::IMREAD_COLOR);

  auto start = high_resolution_clock::now();

  pair<vector<LidarAStar::Gateway>, bool> gateway_path = LidarAStar::AStar::findGatewayPath(global_map_rgb, 
                                                                        start_pose, goal_pose);
  MotionPlan::Vehicle vehicle_1;
  vehicle_1.getLidarAStarPath(start_pose, goal_pose, gateway_path, true);

  auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);

  vehicle_1.printLidarAStarPathOnMap(global_map_rgb);

  resize(global_map_rgb, global_map_rgb, cv::Size(), 2, 2);
  cv::imwrite("/home/popo/LidarAStar/plan_map.bmp", global_map_rgb);                                                               
  
  float duration_cnt = duration.count();
  std::cout << "\nComputation time: "<< duration_cnt/1000<<" ms \n\n";                                                                        
  return 0;
}
