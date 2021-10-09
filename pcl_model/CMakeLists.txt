######################################
## ROS PACKAGE: pcl_model           ##
## AUTHOR: Fran Jurican             ##
######################################
cmake_minimum_required(VERSION 3.0.2)
project(pcl_model)

## compile as C++11
add_compile_options(-std=c++11)

#################################################
## FIND REQUIRED CATKIN PACKAGES AND LIBRARIES ##
#################################################
## catkin packages
find_package(
  catkin REQUIRED COMPONENTS 
  roscpp 
  pcl_ros 
  eigen_conversions 
  pcl_conversions
)

## find libs on system
find_package(Boost COMPONENTS system REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA 10.2 REQUIRED)

#############################################################################
## HEADERS, LIBRARIES AND OTHER CATKIN PACKAGES EXPORTED WITH THIS PACKAGE ##
#############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES CutRobotOut GetPointCloudKinect KinectRegistration 
  CATKIN_DEPENDS roscpp pcl_ros eigen_conversions pcl_conversions
)

#####################################
## HEADERS PATHS (FOR ALL TARGETS) ##
#####################################
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

#####################################
## DEFINITIONS FOR PCL ENVIRONMENT ##
#####################################
add_definitions(${PCL_DEFINITIONS})

###############################
## C++ SHARED LIBRARYS (.so) ##
###############################
add_library(CutRobotOut src/lib/CutRobotOut.cpp)
add_library(GetPointCloudKinect src/lib/GetPointCloudKinect.cpp)
add_library(KinectRegistration src/lib/KinectRegistration.cpp)

#####################
## C++ EXECUTABLES ##
#####################
add_executable(cut_robot src/CutRobotMain.cpp)
add_executable(get_pointcloud_kinect src/GetPointCloudMain.cpp)
add_executable(kinect_registration src/KinectRegistrationMain.cpp)

#####################################################
## LINK TARGETS AGAINST OTHER LIBRARIES OR TARGETS ##
#####################################################
# KinectRegistration .so
target_link_libraries(CutRobotOut ${PCL_LIBRARIES})
target_link_libraries(GetPointCloudKinect ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(KinectRegistration ${PCL_LIBRARIES})

# executables
target_link_libraries(cut_robot ${catkin_LIBRARIES} CutRobotOut)
target_link_libraries(get_pointcloud_kinect ${catkin_LIBRARIES} GetPointCloudKinect)
target_link_libraries(kinect_registration ${catkin_LIBRARIES} KinectRegistration)
