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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Base class for changing device arrays in non-cuda files
// =============================================================================

#ifndef CH_DEVICEUTILS_H_
#define CH_DEVICEUTILS_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/custom_math.h"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

namespace chrono {
namespace fsi {

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
#define U1CAST(x) (uint*)thrust::raw_pointer_cast(&x[0])
#define R1CAST(x) (Real*)thrust::raw_pointer_cast(&x[0])
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
#define LARGE_NUMBER 99999999
#define SMALL_NUMBER -99999999
#define PI 3.1415926535897932384626433832795028841971693993751058f
#define INVPI 0.3183098861837906715377675267450287240689192914809128f
#define EPSILON 1e-8

#define RESOLUTION_LENGTH_MULT 2

// ----------------------------------------------------------------------------
// cutilSafeCall
// CUT_CHECK_ERROR
//
// Legacy CUTIL macros. Currently default to no-ops (TODO)
// ----------------------------------------------------------------------------
#define cudaCheckError()                                                                     \
    {                                                                                        \
        cudaError_t e = cudaGetLastError();                                                  \
        if (e != cudaSuccess) {                                                              \
            printf("Cuda failure %s:%d: '%s'\n", __FILE__, __LINE__, cudaGetErrorString(e)); \
            exit(0);                                                                         \
        }                                                                                    \
    }

// --------------------------------------------------------------------
// GpuTimer
//
/// @brief A template time recorder for cuda events.
/// This utility class encapsulates a simple timer for recording the time between a start and stop event.
// --------------------------------------------------------------------
class GpuTimer {
  public:
    GpuTimer(cudaStream_t stream = 0) : m_stream(stream) {
        cudaEventCreate(&m_start);
        cudaEventCreate(&m_stop);
    }

    ~GpuTimer() {
        cudaEventDestroy(m_start);
        cudaEventDestroy(m_stop);
    }

    void Start() { cudaEventRecord(m_start, m_stream); }
    void Stop() { cudaEventRecord(m_stop, m_stream); }

    float Elapsed() {
        float elapsed;
        cudaEventSynchronize(m_stop);
        cudaEventElapsedTime(&elapsed, m_start, m_stop);
        return elapsed;
    }

  private:
    cudaStream_t m_stream;
    cudaEvent_t m_start;
    cudaEvent_t m_stop;
};

// --------------------------------------------------------------------
// ChDeviceUtils
//
/// This utility class encapsulates a operators on device vectors which might be needed in host files
// --------------------------------------------------------------------
class CH_FSI_API ChDeviceUtils {
  public:
    /// Resizes a thrust vector of Real3 on the device to a specific size
    static void ResizeMyThrust3(thrust::device_vector<Real3>& mThrustVec, int mSize);

    /// Resizes a thrust vector of Real4 on the device to a specific size
    static void ResizeMyThrust4(thrust::device_vector<Real4>& mThrustVec, int mSize);

    /// Fills out a thrust vector of Real4 on the device with a specific Real4
    static void FillMyThrust4(thrust::device_vector<Real4>& mThrustVec, Real4 v);

    /// Clears a thrust vector of Real3 from the device
    static void ClearMyThrustR3(thrust::device_vector<Real3>& mThrustVec);

    /// Clears a thrust vector of Real4 from the device
    static void ClearMyThrustR4(thrust::device_vector<Real4>& mThrustVec);

    /// Clears a thrust vector of unsigned int from the device
    static void ClearMyThrustU1(thrust::device_vector<uint>& mThrustVec);

    /// Appends a Real3 data to a thrust vector of Real3 on the device
    static void PushBackR3(thrust::device_vector<Real3>& mThrustVec, Real3 a3);

    /// Appends a Real4 data to a thrust vector of Real4 on the device
    static void PushBackR4(thrust::device_vector<Real4>& mThrustVec, Real4 a4);

    /// Resizes a thrust vector of Real4 on the device to a specific size
    static void ResizeR3(thrust::device_vector<Real3>& mThrustVec, int size);

    /// Resizes a thrust vector of Real3 on the device to a specific size
    static void ResizeR4(thrust::device_vector<Real4>& mThrustVec, int size);

    /// Resizes a thrust vector of uint on the device to a specific size
    static void ResizeU1(thrust::device_vector<uint>& mThrustVec, int size);

  private:
};
}  // end namespace fsi
}  // end namespace chrono

#endif
