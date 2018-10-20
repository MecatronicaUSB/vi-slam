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

find_package(CUDA)

set(CMAKE_BUILD_TYPE Debug)


if(CUDA_FOUND)
    INCLUDE(FindCUDA)
  # If the package has been found, several variables will
  # be set, you can find the full list with descriptions
  # in the CUDAConfig.cmake file.
  # Print some message showing some of them
  message("******************")
  message("CUDA found")
  message("vi_slam GPU version will be compiled")
  message("******************")
  message(STATUS "CUDA library status:")
  message(STATUS "    toolkit root dir: ${CUDA_TOOLKIT_ROOT_DIR}")
  message(STATUS "    version: ${CUDA_VERSION}")
  message(STATUS "    libraries: ${CUDA_LIBRARIES}")
  message(STATUS "    include path: ${CUDA_INCLUDE_DIRS}")
  message(STATUS "    include path: ${OpenCV_PATH}")


endif(CUDA_FOUND)

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
    ${PROJECT_SOURCE_DIR}/src/DataReader.cpp
    ${PROJECT_SOURCE_DIR}/src/Imu.cpp
    ${PROJECT_SOURCE_DIR}/src/Plus.cpp  
)

#INCLUDE(/usr/local/cuda-8.0/targets/x86_64-linux/include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  #/usr/local/cuda-8.0/targets/x86_64-linux/include
  #${PROJECT_SOURCE_DIR}/include/thirdparty/akaze
  #${CUDASIFT}
)



#add_library(cuda_prof_lib ${cuda_PROFILER})

if(CUDA_FOUND)
  set(vi_slam_GPU_SOURCE_FILES
      ${PROJECT_SOURCE_DIR}/src/MatcherGPU.cpp 
  )
  add_library(vi_slam_libs ${vi_slam_SOURCE_FILES} ${vi_slam_GPU_SOURCE_FILES}) # comentado hasta que se cree librerias
  add_executable(vi_slam ${PROJECT_SOURCE_DIR}/src/main_vi_slamGPU.cpp)
  target_link_libraries(vi_slam vi_slam_libs ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CUDA_LIBRARIES})
  else(CUDA_FOUND)
    message("******************")
    message("CUDA not found")
    message("vi_slam CPU version will be compiled")
    message("******************")
    add_library(vi_slam_libs ${vi_slam_SOURCE_FILES}) 
    add_executable(vi_slam ${PROJECT_SOURCE_DIR}/src/main_vi_slam.cpp)
    target_link_libraries(vi_slam vi_slam_libs ${OpenCV_LIBS} ${catkin_LIBRARIES})

endif(CUDA_FOUND)


#add_executable(vi_slam ${PROJECT_SOURCE_DIR}/src/akaze.cpp)

## IMPORTANT - TO WORK WITH CUDA AND ROS (CUDA INSTALLED IN ANOTHER OPENCV)
string(REPLACE "/opt/ros/kinetic/lib/libopencv_core3.so.3.3;" "" catkin_LIBRARIES "${catkin_LIBRARIES}")

