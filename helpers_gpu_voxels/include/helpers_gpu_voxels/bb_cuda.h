#ifndef BB_CUDA_VOXELS_H
#define BB_CUDA_VOXELS_H

// C++ stl
#include <iostream>
#include <vector>

// Gpu-Voxels
#include <gpu_voxels/GpuVoxels.h>

//Eigen
#include <Eigen/Geometry>

namespace bb_cuda {

    /* Shared ptr for ProbVoxelMap */
    typedef boost::shared_ptr<gpu_voxels::voxelmap::ProbVoxelMap> ProbVoxelMapPtr;

    /* Shared ptr for BitVectorMap */
    typedef boost::shared_ptr<gpu_voxels::voxelmap::BitVectorVoxelMap> BitVectorVoxelMapPtr;

    /* Shared ptr for VoxelCloud */
    typedef boost::shared_ptr<std::vector<Eigen::Vector3f>> VoxelCloudPtr;

    /*
        Get objects VoxelCloud.
        \param map - probabilistic voxel map
        \param leftDown - point (base link frame) of area in which object is detected
        \param rightUp - point (base link frame) of area in which object is detected
        \param sceneOffset - scene offset in Gpu-Voxels map
        \param threshold - threshold for occupancy
        \return objects VoxelCloud, if something went south returns empty VoxelCloud
    */
    VoxelCloudPtr getObjectVoxelCloud(ProbVoxelMapPtr map, Eigen::Vector3f leftDown, Eigen::Vector3f rightUp, 
                        Eigen::Vector3f sceneOffset, Eigen::Vector3f offset = Eigen::Vector3f(0, 0, 0) , float threshold = 0.75);

    /*
        Convert probability value to float.
        \param probab - probability value as signed byte (int8_t)
        \return probability as float in range [0, 1], negative value for unknown
    */
    float convertProbabValueToFloat(int8_t probab);

    /* 
        Check if point is inside map 
        \point - point in meters -> (x, y, z)
        \map - voxel map
        \retune true if point is inside map
    */
    bool checkPoint(Eigen::Vector3f point, ProbVoxelMapPtr map);

    /* 
        Collide maps on CPU, iterates over smaller map (probMap)!
        \probMap - probabilistic map
        \bitMap - bitVector map
        \threshold - threshold for ocupancy
        \return - number of voxels in collision 
    */
    uint32_t collideMapsFastCPU(ProbVoxelMapPtr probMap, BitVectorVoxelMapPtr bitMap, float threshold = 0.75);

    /* 
        Collide maps on GPU, iterates over smaller map (probMap)!
        \probMap - probabilistic map
        \bitMap - bitVector map
        \threshold - threshold for ocupancy
        \blocks - number of CUDA blocks
        \threads - number of threads per block
        \return - number of voxels in collision 
    */
    uint32_t collideMapsFastGPU(ProbVoxelMapPtr probMap, BitVectorVoxelMapPtr bitMap, float threshold = 0.75, int blocks = 2, int threads = 256);

} // namespace bb_cuda

#endif // BB_CUDA_VOXELS_H