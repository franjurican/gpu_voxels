######################################
## ROS PACKAGE: gpu_collision_check ##
## AUTHOR: Fran Jurican             ##
######################################
cmake_minimum_required(VERSION 3.0.2)
project(gpu_collision_check)

## compile as C++11
add_compile_options(-std=c++11)

##################################################
## ENTER PATH TO INSTALLATION DIR OF GPU_VOXELS ##
##################################################
set(GPU_VOXELS_INSTALLATION_DIR "/home/bmaric/Documents/Libraries/gpu-voxels/export")

## extend "CMAKE_SYSTEM_PREFIX_PATH" only in this scope (for gpu_voxels serach path)
## if we extend ENV{CMAKE_PREFIX_PATH}, then gpu_voxels will extend our ROS ws, we don't want that!
list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${GPU_VOXELS_INSTALLATION_DIR})

#################################################
## FIND REQUIRED CATKIN PACKAGES AND LIBRARIES ##
#################################################
## catkin packages
find_package(
  catkin REQUIRED COMPONENTS 
  roscpp 
  helpers_gpu_voxels
  pcl_ros 
  tf2 
  tf2_ros 
  eigen_conversions 
  pcl_conversions
  moveit_ros_planning_interface
  moveit_visual_tools
  std_msgs
  trajectory_msgs
  topp_ros
  message_generation
)

## find libs on system
find_package(Boost COMPONENTS system REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(CUDA 10.2 REQUIRED)
find_package(gpu_voxels REQUIRED)
find_package(icl_core REQUIRED)

#########################
## ROS SERVICE MESSAGE ##
#########################
add_service_files(
  FILES 
  TrajectoryForCollision.srv
)

generate_messages(
  DEPENDENCIES
  trajectory_msgs std_msgs
)

#############################################################################
## HEADERS, LIBRARIES AND OTHER CATKIN PACKAGES EXPORTED WITH THIS PACKAGE ##
#############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RobotAndSceneVoxels CollisionAlgorithmsVoxels
  CATKIN_DEPENDS roscpp helpers_gpu_voxels pcl_ros tf2 tf2_ros eigen_conversions trajectory_msgs message_runtime
)

#####################################
## HEADERS PATHS (FOR ALL TARGETS) ##
#####################################
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${gpu_voxels_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

####################################################
## DEFINITIONS FOR PCL AND GPU-VOXELS ENVIRONMENT ##
####################################################
add_definitions(
  ${PCL_DEFINITIONS} 
  ${icl_core_DEFINITIONS} 
  ${gpu_voxels_DEFINITIONS}
)

###############################
## C++ SHARED LIBRARYS (.so) ##
###############################
add_library(RobotAndSceneVoxels src/gpu_collision/InsertRobotVoxels.cpp src/gpu_collision/InsertSceneVoxels.cpp)
add_library(CollisionAlgorithmsVoxels src/gpu_collision/CollisionAlgorithmsVoxels.cpp)

#####################
## C++ EXECUTABLES ##
#####################
add_executable(broadcast_kinect_TF src/kinects/BroadcastKinectTF.cpp)
add_executable(merge_kinects src/kinects/MergeKinectsMain.cpp src/kinects/MergeKinects.cpp)
add_executable(path_planner src/path_planner/PlanPathMain.cpp)
add_executable(collision_check src/gpu_collision/CollisionMain.cpp)

#####################################################
## LINK TARGETS AGAINST OTHER LIBRARIES OR TARGETS ##
#####################################################
# robot and scene voxels .so
target_link_libraries(RobotAndSceneVoxels 
  ${catkin_LIBRARIES} 
  ${gpu_voxels_LIBRARIES}
)

# collision alghotithms .so
target_link_libraries(CollisionAlgorithmsVoxels 
  ${catkin_LIBRARIES} 
  ${gpu_voxels_LIBRARIES} 
  RobotAndSceneVoxels
)

# gpu_voxels robot working version
target_link_libraries(collision_check
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}  
  ${gpu_voxels_LIBRARIES} 
  RobotAndSceneVoxels 
  CollisionAlgorithmsVoxels
)

# broadcast kinects tf
target_link_libraries(broadcast_kinect_TF 
  ${catkin_LIBRARIES}
)

# merge kinects
target_link_libraries(merge_kinects 
  ${catkin_LIBRARIES}
)

# path planner
target_link_libraries(path_planner
  ${catkin_LIBRARIES} 
  ${icl_core_LIBRARIES} 
  ${gpu_voxels_LIBRARIES} 
  RobotAndSceneVoxels 
)

#############################
## EXECUTABLES FOR INSTALL ##
#############################
install(
  TARGETS broadcast_kinect_TF merge_kinects path_planner collision_check
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########################
## LIBRARIES FOR INSTALL ##
###########################
install(
  TARGETS RobotAndSceneVoxels CollisionAlgorithmsVoxels
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#########################
## HEADERS FOR INSTALL ##
#########################
install(
  DIRECTORY include/${PROJECT_NAME}/gpu_collision
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

######################################################
## URDF, LAUNCH FILES AND CONFIG FILES FOR INSTALL ##
######################################################
install(
  DIRECTORY config launch srv urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
