#ifndef PATH_PLANNER_VOXELS_H
#define PATH_PLANNER_VOXELS_H

// C++ stl
#include <iostream>
#include <vector>

//Eigen
#include <Eigen/Geometry>

// header for bb_cuda
#include <helpers_gpu_voxels/bb_cuda.h>

namespace path_planner_voxels {

    /* 
        Get minimum oriented bounding box of object. 
        \param voxelCloud - voxelCloud of object (minimal number of voxels 10)
        \param transformMatrix - returns transformation matrix of voxelCloud (bounding box)
        \return - bounding box around ORIGIN (min-max point), if something went wrong returns empty vector
    */
    std::vector<Eigen::Vector3f> getMinimumOrientedBoundingBox(bb_cuda::VoxelCloudPtr voxelCloud, Eigen::Affine3f &transformMatrix);

    /*
        Path is always along x-axis and in direction of positive z-axis -> axis are for transformed BB (BB in origin)!
        \param bb - bounding box around ORIGIN (min-max point)
        \param transformMatrix - transformation matrix of bounding box
        \param distance - distance between sensor and object
        \param interpolation - distance for interpolation
        \return - waypoints, if something went wrong returns empty vector
    */
    std::vector<Eigen::Vector3f> generatePathWaypoints(std::vector<Eigen::Vector3f> bb, Eigen::Affine3f transformMatrix, 
                                                                            float distance = 0.1, float interpolation = 0.1);

    /* 
        Generates and visualizes waypoints 
        \param voxelCloud - voxelCloud of object
        \param - returns transformation matrix of object
        \param distance - distance between sensor and object
        \param interpolation - distance for interpolation
        \retrun - waypoints, if something went wrong returns empty vector
    */
    std::vector<Eigen::Vector3f> generateAndVisualizeWaypoints(bb_cuda::VoxelCloudPtr voxelCloud, 
                                        Eigen::Affine3f &transformMatrix, float distance = 0.1, float interpolation = 0.1);

} // namespace path_planner_voxels
#endif // PATH_PLANNER_VOXELS_H