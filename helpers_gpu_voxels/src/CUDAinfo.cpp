/*
Edited by: Fran Juričan
Date: 12.12.2019 
This sample queries the properties of the CUDA device for instalation of GPU-Voxels

**********************************************
* Parts taken from NVIDIA deviceQuery sample *
**********************************************
*/

// C++ stl
#include <iostream>
#include <memory>
#include <string>

// CUDA
#include <cuda_runtime.h>

// taken from NVIDIA deviceQuery sample
#include <helpers_gpu_voxels/cuda/helper_cuda.h>

// works for CUDA version > 5.0 !!!!!!
int main(int argc, char **argv) {

  int deviceCount = 0;
  cudaError_t error_id = cudaGetDeviceCount(&deviceCount);

  if (error_id != cudaSuccess) {
    printf("cudaGetDeviceCount returned %d\n-> %s\n",
           static_cast<int>(error_id), cudaGetErrorString(error_id));
    printf("Result = FAIL\n");
    exit(EXIT_FAILURE);
  }

  // this function call returns 0 if there are no CUDA capable devices.
  if (deviceCount == 0) {
    printf("\nThere are no available device that support CUDA\n");
  } else {
    printf("\nDetected %d CUDA Capable device(s)\n", deviceCount);
  }

  int dev, driverVersion = 0, runtimeVersion = 0;

  for (dev = 0; dev < deviceCount; ++dev) {
    cudaSetDevice(dev);
    cudaDeviceProp deviceProp;
    cudaGetDeviceProperties(&deviceProp, dev);

    printf("\nDevice %d:                                      \"%s\"\n", dev+1, deviceProp.name);

    // driver version
    cudaDriverGetVersion(&driverVersion);
    cudaRuntimeGetVersion(&runtimeVersion);

    // driver
    printf("CUDA Driver Version / Runtime Version:         %d.%d / %d.%d\n", driverVersion/1000, (driverVersion%100)/10,
           runtimeVersion / 1000, (runtimeVersion % 100) / 10);

    // CUDA capability   
    printf("CUDA Capability Major/Minor version number:    %d.%d\n", deviceProp.major, deviceProp.minor);

    // GPU memory   
    printf("Total amount of global memory:                 %.0f MBytes (%llu bytes)\n" , 
          static_cast<float>(deviceProp.totalGlobalMem / 1048576.0f), (unsigned long long)deviceProp.totalGlobalMem);
    
    // CUDA cores
    printf("(%1d) Multiprocessors, (%3d) CUDA Cores/MP:      %d CUDA Cores\n", deviceProp.multiProcessorCount,
           _ConvertSMVer2Cores(deviceProp.major, deviceProp.minor), 
           _ConvertSMVer2Cores(deviceProp.major, deviceProp.minor)*deviceProp.multiProcessorCount);

    // clock rate
    printf("GPU Max Clock rate:                            %.0f MHz (%0.2f GHz)\n", deviceProp.clockRate * 1e-3f,
         deviceProp.clockRate * 1e-6f);

    // bus size
    printf("Memory Bus Width:                              %d-bit\n", deviceProp.memoryBusWidth);

    // threads per multiprocessor      
    printf("Maximum number of threads per multiprocessor:  %d\n", deviceProp.maxThreadsPerMultiProcessor);

    // threads per block !!
    printf("Maximum number of threads per block:           %d\n", deviceProp.maxThreadsPerBlock);

    // size thread block
    printf("Max dimension size of a thread block (x,y,z): (%d, %d, %d)\n", deviceProp.maxThreadsDim[0], 
          deviceProp.maxThreadsDim[1], deviceProp.maxThreadsDim[2]);
  }

  // CUDA is working
  printf("\n\t\t    *******************\n");
  printf("\t\t    * CUDA works fine *\n");
  printf("\t\t    *******************\n");
  
  return 0;
}