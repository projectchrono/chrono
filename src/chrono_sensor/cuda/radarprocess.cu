#include  <cuda.h>
#include <iostream>
#include "radarprocess.cuh"

namespace chrono{
namespace sensor{

// Converts a depth and intensity buffer to an XZY and intensity buffer
__global__ void radar_pointcloud_from_depth_kernel(float* imgIn,
                                                   float* imgOut,
                                                   int w,
                                                   int h,
                                                   float hfov,
                                                   float max_v_angle,
                                                   float min_v_angle) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index < w * h) {
        int hIndex = index % w;
        int vIndex = index / w;

        float vAngle = (vIndex / (float)(h)) * (max_v_angle - min_v_angle) + min_v_angle;

        float hAngle = (hIndex / (float)(w)) * hfov - hfov / 2.;

        float range = imgIn[6 * index];
        float proj_xy = range * cosf(vAngle);
        float x = proj_xy * cosf(hAngle);
        float y = proj_xy * sinf(hAngle);
        float z = range * sinf(vAngle);
        imgOut[8 * index] = x;
        imgOut[8 * index + 1] = y;
        imgOut[8 * index + 2] = z;
        imgOut[8 * index + 3] = imgIn[6 * index + 1];
        imgOut[8 * index + 4] = imgIn[6 * index + 2];
        imgOut[8 * index + 5] = imgIn[6 * index + 3];
        imgOut[8 * index + 6] = imgIn[6 * index + 4];
        imgOut[8 * index + 7] = imgIn[6 * index + 5];
    //        printf("%f %f %f %f %f %f\n", imgIn[6 * index], imgIn[6 * index + 1], imgIn[6 * index + 2], imgIn[6 * index + 3], imgIn[6 * index + 4], imgIn[6 * index + 5]);
    //        printf("%f %f %f\n",imgIn[6 * index] , imgIn[6 * index + 5], imgIn[6 * index + 1]);
    //        printf("%f %f %f\n", x, y, z);
    //        printf("%f %f\n", imgOut[8 * index + 7],imgOut[8 * index + 3]);
    }
}


void cuda_radar_pointcloud_from_depth(void* bufIn,
    void* bufOut,
    int width,
    int height,
    float hfov,
    float max_v_angle,
    float min_v_angle,
    CUstream& stream){
const int nThreads = 512;
int nBlocks = (width * height + nThreads - 1) / nThreads;
radar_pointcloud_from_depth_kernel<<<nBlocks, nThreads, 0, stream>>>((float*)bufIn, (float*)bufOut, width, height,
                                       hfov, max_v_angle, min_v_angle);
}

}
}