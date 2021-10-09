#####################################
## ROS PACKAGE: helpers_gpu_voxels ##
## AUTHOR: Fran Jurican            ##
#####################################
cmake_minimum_required(VERSION 3.0.2)
project(helpers_gpu_voxels)

## compile as C++11
add_compile_options(-std=c++11)
list(APPEND CUDA_NVCC_FLAGS "-std=c++11")

##################################################
## ENTER PATH TO INSTALLATION DIR OF GPU_VOXELS ##
##################################################
set(GPU_VOXELS_INSTALLATION_DIR "/home/bmaric/Documents/Libraries/gpu-voxels/export")

## extend "CMAKE_SYSTEM_PREFIX_PATH" only in this scope (for gpu_voxels serach path)
## if we extend CMAKE_PREFIX_PATH, then gpu_voxels will extend our ROS ws, we don't want that!
list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${GPU_VOXELS_INSTALLATION_DIR})

#################################################
## FIND REQUIRED CATKIN PACKAGES AND LIBRARIES ##
#################################################
## catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros urdf 
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2)

## find libs on system
find_package(PCL 1.7 REQUIRED)
## clear PCL external definitions (can cause errors with nvcc)
set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "" )
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(CUDA 10.2 REQUIRED)
find_package(gpu_voxels REQUIRED)
find_package(icl_core REQUIRED)

#############################################################################
## HEADERS, LIBRARIES AND OTHER CATKIN PACKAGES EXPORTED WITH THIS PACKAGE ##
#############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES TimerWrapper PathPlannerVoxels BoundingBoxCUDA 
  CATKIN_DEPENDS roscpp
)

#####################################
## HEADERS PATHS (FOR ALL TARGETS) ##
#####################################
include_directories( 
  include 
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS} 
  ${gpu_voxels_INCLUDE_DIRS} 
  ${CUDA_INCLUDE_DIRS}
)

############################################
## DEFINITIONS FOR GPU-VOXELS ENVIRONMENT ##
############################################
add_definitions(
  ${icl_core_DEFINITIONS} 
  ${gpu_voxels_DEFINITIONS}
)

###############################
## C++ SHARED LIBRARIES (.so) ##
###############################
add_library(TimerWrapper src/TimerWrapper.cpp)
add_library(PathPlannerVoxels src/PathPlannerVoxels.cpp)

#####################################
## CUDA C++ SHARED LIBRARIES (.so) ##
#####################################
cuda_add_library(BoundingBoxCUDA src/BBcuda.cu)

#####################
## C++ EXECUTABLES ##
#####################
add_executable(urdf_parser src/URDFparser.cpp)
add_executable(cuda_info src/CUDAinfo.cpp)

#####################################################
## LINK TARGETS AGAINST OTHER LIBRARIES OR TARGETS ##
#####################################################
target_link_libraries(TimerWrapper ${catkin_LIBRARIES})
target_link_libraries(PathPlannerVoxels ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(urdf_parser ${catkin_LIBRARIES})
target_link_libraries(cuda_info ${catkin_LIBRARIES} ${CUDA_LIBRARIES})

target_link_libraries(BoundingBoxCUDA 
  ${catkin_LIBRARIES} 
  ${CUDA_LIBRARIES}
  ${gpu_voxels_LIBRARIES}
  ${icl_core_LIBRARIES}   
)

################################
## PYTHON SCRIPTS FOR INSTALL ##
################################
catkin_install_python(
  PROGRAMS scripts/boundingBoxGenerator.py scripts/pickAndPlace.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############################
## EXECUTABLES FOR INSTALL ##
#############################
install(
  TARGETS urdf_parser cuda_info
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########################
## LIBRARIES FOR INSTALL ##
###########################
install(
  TARGETS TimerWrapper PathPlannerVoxels BoundingBoxCUDA
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#########################
## HEADERS FOR INSTALL ##
#########################
install(
  DIRECTORY include/${PROJECT_NAME}/timer 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES include/${PROJECT_NAME}/bb_cuda.h include/${PROJECT_NAME}/path_planner_voxels.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#####################################################
## URDF, LAUNCH FILES AND CONFIG FILES FOR INSTALL ##
#####################################################
install(
  DIRECTORY urdf launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
