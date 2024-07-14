Developed by: Po-Jen Wang
March 12th, 2021
pwang37@ucsc.edu

Demo program for Path/Motion Planning algorithm "Lidar A*" based on my master thesis:
https://www.researchgate.net/publication/342721045_LIDAR_A_An_Online_Visibility-Based_Decomposition_and_Search_Approach_for_Real-time_Autonomous_Vehicle_Motion_Planning

Language: C++

This is a new approach to the classic path planning problem in robotics. It determines a path between two points on the map by performing simulated Lidar explorations. This early version program is still actively being updated and modified. Please avoid making this program open-source and use the resulting path with extreme caution (not guaranteed bug free)!

Program Dependencies: 
OpenCV 4 (you need to link the locally installed opencv directory in the CMakeLists.txt)

Prereq:
In demo.cpp, change the file paths for the input (cv::imread(..)) and output(cv::imwrite(..)) maps. 
You can modify the start and goal pose as well.

To run demo program:
first go to 

~/LidarAStar/build

then in terminal:

cmake ..
make
./demo

An output map will be generated in the directory specified in demo.cpp
Detailed path info is stored in the "gateway_path", vector of gateway class member
