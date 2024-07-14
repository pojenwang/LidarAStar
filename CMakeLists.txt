# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(Motion_Planning)

# Link this to your opencv installation directory
SET(OpenCV_DIR /home/popo/opencv4/build)

include_directories(source)
add_library(LidarAStar source/LidarAStar.cpp)
add_library(LocalAnalysis source/LocalAnalysis.cpp)
add_library(MotionPlanning source/MotionPlanning.cpp)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Declare the executable target built from your sources
add_executable(demo demo.cpp)

# Link your application with OpenCV libraries
target_link_libraries(demo PRIVATE ${OpenCV_LIBS})
target_link_libraries(LidarAStar PUBLIC ${OpenCV_LIBS})
target_link_libraries(LocalAnalysis PUBLIC ${OpenCV_LIBS})
target_link_libraries(MotionPlanning PUBLIC ${OpenCV_LIBS})
target_link_libraries(demo PUBLIC LidarAStar)
target_link_libraries(demo PUBLIC MotionPlanning)
target_link_libraries(LidarAStar PUBLIC LocalAnalysis)
target_link_libraries(MotionPlanning PUBLIC LidarAStar)
target_link_libraries(LocalAnalysis PUBLIC LidarAStar)


