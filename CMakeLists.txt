cmake_minimum_required(VERSION 2.8.3)
project(vi_slam)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

# Opencv Library (local)
set(OpenCV_INCLUDE_DIRS /usr/local/)
set(OpenCV_LIBS /usr/local/lib/libopencv_core.so)

# Required libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  image_transport
  cv_bridge
  message_generation
)
find_package(OpenCV REQUIRED
NO_MODULE
PATHS /usr/local
NO_DEFAULT_PATH)


set(CMAKE_BUILD_TYPE Debug)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  visualization_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vi_slam
   CATKIN_DEPENDS roscpp visualization_msgs message_runtime
#  DEPENDS system_lib
)

# Source files
set(vi_slam_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/GroundTruth.cpp
    ${PROJECT_SOURCE_DIR}/src/Visualizer.cpp
    ${PROJECT_SOURCE_DIR}/src/ImageReader.cpp
    ${PROJECT_SOURCE_DIR}/src/Matcher.cpp
    
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_library(vi_slam_libs ${vi_slam_SOURCE_FILES}) # comentado hasta que se cree librerias

add_executable(vi_slam ${PROJECT_SOURCE_DIR}/src/main_vi_slam.cpp)

## IMPORTANT - TO WORK WITH CUDA AND ROS (CUDA INSTALLED IN ANOTHER OPENCV)
string(REPLACE "/opt/ros/kinetic/lib/libopencv_core3.so.3.3;" "" catkin_LIBRARIES "${catkin_LIBRARIES}")

target_link_libraries(vi_slam vi_slam_libs ${OpenCV_LIBS} ${catkin_LIBRARIES})
