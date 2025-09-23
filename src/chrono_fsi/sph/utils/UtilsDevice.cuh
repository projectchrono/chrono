// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu, Radu Serban
// =============================================================================
//
// Utilities for changing device arrays in non-cuda files
// =============================================================================

#ifndef CH_UTILS_DEVICE_H
#define CH_UTILS_DEVICE_H

#include <cuda_runtime.h>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "chrono/core/ChTypes.h"

#include "chrono_fsi/sph/math/CustomMath.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_utils
/// @{

// ----------------------------------------------------------------------------
// Short-hand notation
// ----------------------------------------------------------------------------

#define mF2 make_float2
#define mF3 make_float3
#define mF4 make_float4
#define mR2 make_Real2
#define mR3 make_Real3
#define mR4 make_Real4

#define mI2 make_int2
#define mI3 make_int3
#define mI4 make_int4

#define mU3 make_uint3

#define F1CAST(x) (float*)thrust::raw_pointer_cast(&x[0])
#define D1CAST(x) (double*)thrust::raw_pointer_cast(&x[0])
#define BCAST(x) (bool*)thrust::raw_pointer_cast(&x[0])

#define I1CAST(x) (int*)thrust::raw_pointer_cast(&x[0])
// This is not 32 uints but rather a single uint32_t
#define UINT_32CAST(x) (uint32_t*)thrust::raw_pointer_cast(&x[0])
// This is not 32 ints but rather a single int32_t
#define INT_32CAST(x) (int32_t*)thrust::raw_pointer_cast(&x[0])
#define mI2CAST(x) (int2*)thrust::raw_pointer_cast(&x[0])
#define mI3CAST(x) (int3*)thrust::raw_pointer_cast(&x[0])
#define mI4CAST(x) (int4*)thrust::raw_pointer_cast(&x[0])
#define U1CAST(x) (uint*)thrust::raw_pointer_cast(&x[0])
#define U2CAST(x) (uint2*)thrust::raw_pointer_cast(&x[0])
#define U3CAST(x) (uint3*)thrust::raw_pointer_cast(&x[0])
#define U4CAST(x) (uint4*)thrust::raw_pointer_cast(&x[0])

#define LU1CAST(x) (unsigned long int*)thrust::raw_pointer_cast(&x[0])
#define R1CAST(x) (Real*)thrust::raw_pointer_cast(&x[0])
#define mR2CAST(x) (Real2*)thrust::raw_pointer_cast(&x[0])
#define mR3CAST(x) (Real3*)thrust::raw_pointer_cast(&x[0])
#define mR4CAST(x) (Real4*)thrust::raw_pointer_cast(&x[0])
#define TCAST(x) thrust::raw_pointer_cast(x.data())
#define mR3BY3CAST(x) (Real3By3*)thrust::raw_pointer_cast(&x[0])

// ----------------------------------------------------------------------------
// editor stuff
// ----------------------------------------------------------------------------
#ifdef __CDT_PARSER__
    #define __host__
    #define __device__
    #define __global__
    #define __constant__
    #define __shared__
    #define CUDA_KERNEL_DIM(...) ()
#else
    #define CUDA_KERNEL_DIM(...) << <__VA_ARGS__>>>
#endif

// ----------------------------------------------------------------------------

#define cudaMallocErrorFlag(error_flag_D)                \
    {                                                    \
        cudaMalloc((void**)&error_flag_D, sizeof(bool)); \
    }

#define cudaFreeErrorFlag(error_flag_D) \
    {                                   \
        cudaFree(error_flag_D);         \
    }

#define cudaResetErrorFlag(error_flag_D)                                               \
    {                                                                                  \
        bool error_flag_H = false;                                                     \
        cudaMemcpy(error_flag_D, &error_flag_H, sizeof(bool), cudaMemcpyHostToDevice); \
    }

#define cudaCheckErrorFlag(error_flag_D, kernel_name)                                                        \
    {                                                                                                        \
        bool error_flag_H;                                                                                   \
        cudaDeviceSynchronize();                                                                             \
        cudaMemcpy(&error_flag_H, error_flag_D, sizeof(bool), cudaMemcpyDeviceToHost);                       \
        if (error_flag_H) {                                                                                  \
            char buffer[256];                                                                                \
            sprintf(buffer, "Error flag intercepted in %s:%d from %s", __FILE__, __LINE__, kernel_name);     \
            printf("%s\n", buffer);                                                                          \
            throw std::runtime_error(buffer);                                                                \
        }                                                                                                    \
        cudaError_t e = cudaGetLastError();                                                                  \
        if (e != cudaSuccess) {                                                                              \
            char buffer[256];                                                                                \
            sprintf(buffer, "CUDA failure in %s:%d Message: %s", __FILE__, __LINE__, cudaGetErrorString(e)); \
            printf("%s\n", buffer);                                                                          \
            throw std::runtime_error(buffer);                                                                \
        }                                                                                                    \
    }

#define cudaCheckError()                                                                                     \
    {                                                                                                        \
        cudaDeviceSynchronize();                                                                             \
        cudaError_t e = cudaGetLastError();                                                                  \
        if (e != cudaSuccess) {                                                                              \
            char buffer[256];                                                                                \
            sprintf(buffer, "CUDA failure in %s:%d Message: %s", __FILE__, __LINE__, cudaGetErrorString(e)); \
            printf("%s\n", buffer);                                                                          \
            throw std::runtime_error(buffer);                                                                \
        }                                                                                                    \
    }

#define cudaThrowError(message)                                                            \
    {                                                                                      \
        char buffer[256];                                                                  \
        sprintf(buffer, "CUDA failure in %s:%d Message: %s", __FILE__, __LINE__, message); \
        printf("%s\n", buffer);                                                            \
        throw std::runtime_error(buffer);                                                  \
    }

// ----------------------------------------------------------------------------

/// Compute number of blocks and threads for calculation on GPU.
/// This function calculates the number of blocks and threads for a given number of elements based on the blockSize.
void computeGridSize(uint n,           ///< total number of elements
                     uint blockSize,   ///< block size (threads per block)
                     uint& numBlocks,  ///< number of blocks [output]
                     uint& numThreads  ///< number of threads [output]
);

// ----------------------------------------------------------------------------

/// Time recorder for cuda events.
/// This utility class encapsulates a simple timer for recording the time between a start and stop event.
class GpuTimer {
  public:
    GpuTimer(cudaStream_t stream = 0);
    ~GpuTimer();

    /// Record the start time.
    void Start();

    /// Record the stop time.
    void Stop();

    /// Return the elapsed time.
    float Elapsed();

  private:
    cudaStream_t m_stream;
    cudaEvent_t m_start;
    cudaEvent_t m_stop;
};

/// @} fsisph_utils

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
