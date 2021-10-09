// C++ stl
#include <iostream>

// CUDA thrust
#include <thrust/reduce.h>
#include <thrust/device_vector.h>

// Gpu-Voxels
#include <gpu_voxels/helpers/common_defines.h>

// ROS
#include <ros/ros.h>

// header
#include <helpers_gpu_voxels/bb_cuda.h>

using namespace gpu_voxels;

namespace bb_cuda
{
    // CUDA kerenel for colliding
    __global__
    void collideMapsKernel(int8_t *probMap, uint64_t *bitMap, uint32_t voxelNum, uint16_t elementsPerThread, int8_t threshold, uint16_t *out)
    {
        // linear index, count and map offset
        uint linearId = blockDim.x*blockIdx.x + threadIdx.x;
        uint16_t count = 0;
        uint32_t offset = linearId*elementsPerThread, k;

        // detect collision
        for(int i = 0; i < elementsPerThread; i++)
        {
            if((i + offset) < voxelNum)
            {
                if(probMap[i + offset] > threshold)
                {
                    k = 4*(i + offset);

                    if((bitMap[k] != 0) || (bitMap[k + 1] != 0) || (bitMap[k + 2] != 0) || (bitMap[k + 3] != 0))
                        count++;

                }
            }
            else
            {
                break;
            } 
        }

        out[linearId] = count;
    }
}

bb_cuda::VoxelCloudPtr bb_cuda::getObjectVoxelCloud(bb_cuda::ProbVoxelMapPtr map, Eigen::Vector3f leftDown, 
                            Eigen::Vector3f rightUp, Eigen::Vector3f sceneOffset, Eigen::Vector3f offset, float threshold)
{
    // local vars
    uint32_t indexLinear, dimX, dimY, mapBytesOnDevice;
    int8_t *mapValuesInt;
    float voxelSize, probabValueFloat, xm, ym, zm;
    Vector3ui ld, ur;
    bb_cuda::VoxelCloudPtr voxelCloud(new std::vector<Eigen::Vector3f>);

    // map size on device, voxel size and map dimension (voxels)
    mapBytesOnDevice = map->getMemoryUsage();
    voxelSize = map->getVoxelSideLength();
    dimX = map->getDimensions().x;
    dimY = map->getDimensions().y;

    // get search area in map frame
    leftDown += sceneOffset;
    rightUp += sceneOffset;

    // check area
    if(!bb_cuda::checkPoint(leftDown, map))
    {
        ROS_ERROR("Tocka \"leftDown\" je izvan dimenzija mape. Izalazim iz trazenja VoxelCloud-a objekta!");
        return voxelCloud;
    } 
    else if(!bb_cuda::checkPoint(rightUp, map))
    {
        ROS_ERROR("Tocka \"upRight\" je izvan dimenzija mape. Izalazim iz trazenja VoxelCloud-a objekta!");
        return voxelCloud;
    }

    // search area in voxels
    leftDown = leftDown/voxelSize;
    rightUp = rightUp/voxelSize;
    ld.x = (uint32_t)leftDown.x();
    ld.y = (uint32_t)leftDown.y();
    ld.z = (uint32_t)leftDown.z();
    ur.x = (uint32_t)rightUp.x();
    ur.y = (uint32_t)rightUp.y();
    ur.z = (uint32_t)rightUp.z();

    // search area info
    ROS_INFO_STREAM("Podrucje trazenja VoxelClud-a max.: " << "(x, y, z) = (" << ld.x << ", " << ld.y << ", " << ld.z << ")");
    ROS_INFO_STREAM("Podrucje trazenja VoxelClud-a min.: " << "(x, y, z) = (" << ur.x << ", " << ur.y << ", " << ur.z << ")");
    
    // get map from device to host
    mapValuesInt = new int8_t[mapBytesOnDevice];
    cudaMemcpy(mapValuesInt, map->getConstVoidDeviceDataPtr(), map->getMemoryUsage(), cudaMemcpyDeviceToHost);

    // find objects VoxelCloud
    for(uint32_t x = ld.x; x <= ur.x; x++)
        for(uint32_t y = ld.y; y <= ur.y; y++)
            for(uint32_t z = ld.z; z <= ur.z; z++) 
            {
                // linear index and probability as float
                indexLinear = x + dimX*y + dimX*dimY*z;
                probabValueFloat = bb_cuda::convertProbabValueToFloat(mapValuesInt[indexLinear]);

                // only occupied voxels
                if(probabValueFloat > threshold)
                {
                   /* // if voxel is on edge, then exit
                    if ((x == ld.x) || (x == ur.x) || (y == ld.y) || (y == ur.y) || (z == ld.z) || (z == ur.z))
                    {
                        ROS_ERROR("Objekt se nalazi izvan prostora trazenja. Izlazim iz trazenja VoxelCloud-a objekta!!");
                        voxelCloud->clear();
                        return voxelCloud;
                    }*/

                    xm = x*voxelSize - sceneOffset.x() - offset.x();
                    ym = y*voxelSize - sceneOffset.y() - offset.y();
                    zm = z*voxelSize - sceneOffset.z() - offset.z();
                    voxelCloud->push_back(Eigen::Vector3f(xm, ym, zm));
                }
            }

    
    if(voxelCloud->size() == 0)
        ROS_WARN("Prostor trazenja je PRAZAN!");
    else
        ROS_INFO_STREAM("Velicina VoxelCloud-a objekta: " << voxelCloud->size());

    // free heap
    delete mapValuesInt;

    return voxelCloud;
}

float bb_cuda::convertProbabValueToFloat(int8_t probab)
{
    int8_t max = gpu_voxels::MAX_PROBABILITY;
    int8_t min = gpu_voxels::MIN_PROBABILITY;

    return ((float)(probab - min))/(max - min);
}

bool bb_cuda::checkPoint(Eigen::Vector3f point, bb_cuda::ProbVoxelMapPtr map)
{
    float maxX, maxY, maxZ;

    maxX = map->getMetricDimensions().x;
    maxY = map->getMetricDimensions().y;
    maxZ = map->getMetricDimensions().z;

    if((point.x() >= 0) && (point.x() < maxX) && (point.y() >= 0) && (point.y() < maxY) && (point.z() >= 0) && (point.z() < maxZ))
        return true;
    else
        return false;   
}

uint32_t bb_cuda::collideMapsFastCPU(bb_cuda::ProbVoxelMapPtr probMap, bb_cuda::BitVectorVoxelMapPtr bitMap, float threshold)
{
    int8_t *buffProb, cutOff;
    uint32_t voxelNum, sizeBytes, collNum = 0, k;
    uint64_t *buffBit;

    // get probability (one byte) -> [-128, 127]
    cutOff = gpu_voxels::ProbabilisticVoxel::floatToProbability(threshold);

    // number of voxels in map and map size in bytes
    voxelNum = probMap->getVoxelMapSize();
    sizeBytes = probMap->getMemoryUsage();

    // prob map -> one byte per voxel, bit vector map 256 bits (32 bytes) per voxel
    buffProb = new int8_t[sizeBytes]; 
    buffBit = new uint64_t[sizeBytes*4];

    // get data
    cudaMemcpy((void *)buffProb, probMap->getConstVoidDeviceDataPtr(), sizeBytes, cudaMemcpyDeviceToHost);
    cudaMemcpy((void *)buffBit, bitMap->getConstVoidDeviceDataPtr(), sizeBytes*32, cudaMemcpyDeviceToHost);

    for(uint32_t i = 0; i < voxelNum; i++)
    {
        if(buffProb[i] > cutOff)
        {
            // for bit map take every 256 bits -> 32 bytes -> 4 uint64_t
            k = i*4;

            // collision happens if voxel in bit vector map is not free
            if((buffBit[k] != 0) || (buffBit[k + 1] != 0) || (buffBit[k + 2] != 0) || (buffBit[k + 3] != 0))
                collNum++;
        }
    }

    //free heap
    delete buffProb;
    delete buffBit;

    return collNum;
}

uint32_t bb_cuda::collideMapsFastGPU(ProbVoxelMapPtr probMap, BitVectorVoxelMapPtr bitMap, float threshold, int blocks, int threads)
{
    // ptrs to device data 
    int8_t *ptrProbMap = (int8_t *)probMap->getVoidDeviceDataPtr();
    uint16_t *outDevice;
    uint64_t *ptrBitMap = (uint64_t *)bitMap->getVoidDeviceDataPtr();

    // ptr to host data
    uint16_t *outHost;

    // threshold
    int8_t probThreshold = gpu_voxels::ProbabilisticVoxel::floatToProbability(threshold);

    // number of voxels in map
    uint32_t voxelNum = probMap->getVoxelMapSize();

    // voxels per thread
    uint16_t voxelsPerThread = voxelNum/(blocks*threads) + 1;
    
    // allocate device and host memory
    cudaMalloc(&outDevice, blocks*threads*sizeof(uint16_t));
    outHost = new uint16_t[blocks*threads];

    // get collision
    bb_cuda::collideMapsKernel<<<blocks, threads>>>(ptrProbMap, ptrBitMap, voxelNum, voxelsPerThread, probThreshold, outDevice);

    // copy data to host
    cudaMemcpy(outHost, outDevice, blocks*threads*sizeof(uint16_t), cudaMemcpyDeviceToHost);

    // get number of voxels in collision
    uint32_t numberOfVoxelsInCollision = 0;
    for(int i = 0; i < blocks*threads; i++)
        numberOfVoxelsInCollision += outHost[i];

    // free memory
    cudaFree(outDevice);
    delete outHost;

    return numberOfVoxelsInCollision;
}
