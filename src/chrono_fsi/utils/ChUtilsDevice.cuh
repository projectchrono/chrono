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

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

// ----------------------------------------------------------------------------
// Short-hand notation
// ----------------------------------------------------------------------------
typedef unsigned int uint;

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
#define mI2CAST(x) (int2*)thrust::raw_pointer_cast(&x[0])
#define mI4CAST(x) (int4*)thrust::raw_pointer_cast(&x[0])
#define U1CAST(x) (uint*)thrust::raw_pointer_cast(&x[0])
#define U2CAST(x) (uint2*)thrust::raw_pointer_cast(&x[0])
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
// Values
// ----------------------------------------------------------------------------

#define INVPI 0.3183098861837906715377675267450287240689192914809128f
#define EPSILON 1e-8

#define RESOLUTION_LENGTH_MULT 2.0
//#define RESOLUTION_LENGTH_MULT 3.0

#define cudaCheckError()                                                                     \
    {                                                                                        \
        cudaError_t e = cudaGetLastError();                                                  \
        if (e != cudaSuccess) {                                                              \
            printf("Cuda failure %s:%d: '%s'\n", __FILE__, __LINE__, cudaGetErrorString(e)); \
            exit(0);                                                                         \
        }                                                                                    \
    }

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

/// Utilities for thrust device vectors.
class CH_FSI_API ChUtilsDevice {
  public:
    /// Fills out a thrust vector of Real3 on the device with a specific Real3 value.
    static void FillVector(thrust::device_vector<Real3>& vector, const Real3& value);

    /// Fills out a thrust vector of Real4 on the device with a specific Real4 value.
    static void FillVector(thrust::device_vector<Real4>& vector, const Real4& value);

    /// Error check.
    static void Sync_CheckError(bool* isErrorH, bool* isErrorD, std::string carshReport);
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
