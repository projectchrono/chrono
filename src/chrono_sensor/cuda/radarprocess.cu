#include  <cuda.h>
#include <iostream>
#include "radarprocess.cuh"

namespace chrono{
namespace sensor{


//__global__ void radar_angle_kernel(float* imgIn,
//                                   float* imgOut,
//                                   int w,
//                                   int h,
//                                   float hfov,
//                                   float max_v_angle,
//                                   float min_v_angle){
//    int index = blockDim.x * blockIdx.x + threadIdx.x;                                    
//    if (index < w * h){
//        int hIndex = index % w;
//        int vIndex = index / w;
//
//        float azimuth = (hIndex / (float)(w)) * hfov - hfov /  2.;
//        float elevation = (vIndex / (float)(h)) * (max_v_angle - min_v_angle) + min_v_angle;
//
//        imgOut[8 * index] = imgIn[6 * index];
//        imgOut[8 * index + 1] = azimuth;
//        imgOut[8 * index + 2] = elevation;
//        imgOut[8 * index + 3] = imgIn[6 * index + 2];
//        imgOut[8 * index + 4] = imgIn[6 * index + 3];
//        imgOut[8 * index + 5] = imgIn[6 * index + 4];
//        imgOut[8 * index + 6] = imgIn[6 * index + 1];
//        imgOut[8 * index + 7] = imgIn[6 * index + 5];
//    }
//}

// Converts a depth and intensity buffer to an XZY and intensity buffer
__global__ void radar_pointcloud_from_angles_kernel(float* imgIn,
                                                   float* imgOut,
                                                   int w,
                                                   int h,
                                                   float hfov,
                                                   float vfov) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index < w * h) {
//        int hIndex = index % w;
//        int vIndex = index / w;
//
//        float vAngle = (vIndex / (float)(h)) * (max_v_angle - min_v_angle) + min_v_angle;
//        float hAngle = (hIndex / (float)(w)) * hfov - hfov / 2.;

        float range = imgIn[8 * index];
        float azimuth = imgIn[8 * index + 1];
        float elevation = imgIn[8 * index + 2];
        float proj_xy = range * cosf(elevation);
        float x = proj_xy * cosf(azimuth);
        float y = proj_xy * sinf(azimuth);
        float z = range * sinf(elevation);
        imgOut[8 * index] = x;
        imgOut[8 * index + 1] = y;
        imgOut[8 * index + 2] = z;
        imgOut[8 * index + 3] = imgIn[8 * index + 3];
        imgOut[8 * index + 4] = imgIn[8 * index + 4];
        imgOut[8 * index + 5] = imgIn[8 * index + 5];
        imgOut[8 * index + 6] = imgIn[8 * index + 6];
        imgOut[8 * index + 7] = imgIn[8 * index + 7];
    }
}


void cuda_radar_pointcloud_from_angles(void* bufIn,
                                       void* bufOut,
                                       int width,
                                       int height,
                                       float hfov,
                                       float vfov,
                                       CUstream& stream){
    const int nThreads = 512;
    int nBlocks = (width * height + nThreads - 1) / nThreads;
    radar_pointcloud_from_angles_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (float*)bufOut, width, height,
                                       hfov, vfov);
}

//void cuda_radar_pointcloud(void* bufIn,
//                           void* bufOut,
//                           )

}
}