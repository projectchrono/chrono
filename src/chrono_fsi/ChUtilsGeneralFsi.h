/*
 * SPHCudaUtils.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */
// ****************************************************************************
// This file contains miscellaneous macros and utilities used in the SPH code.
// ****************************************************************************
#ifndef SPH_CUDA_UTILS_H
#define SPH_CUDA_UTILS_H

// ----------------------------------------------------------------------------
// CUDA headers
// ----------------------------------------------------------------------------
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_functions.h>
#include <device_launch_parameters.h>
#include "chrono_fsi/custom_cutil_math.h"
#include <ctime>

#include <stdio.h>

// ----------------------------------------------------------------------------
// Short-hand notation
// ----------------------------------------------------------------------------
namespace chrono {
namespace fsi {


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

#define F1CAST(x) (float*) thrust::raw_pointer_cast(&x[0])
#define D1CAST(x) (double*) thrust::raw_pointer_cast(&x[0])
#define BCAST(x) (bool*) thrust::raw_pointer_cast(&x[0])

#define I1CAST(x) (int*) thrust::raw_pointer_cast(&x[0])
#define mI2CAST(x) (int2*) thrust::raw_pointer_cast(&x[0])
#define U1CAST(x) (uint*) thrust::raw_pointer_cast(&x[0])
#define R1CAST(x) (Real*) thrust::raw_pointer_cast(&x[0])
#define mR3CAST(x) (Real3*) thrust::raw_pointer_cast(&x[0])
#define mR4CAST(x) (Real4*) thrust::raw_pointer_cast(&x[0])
#define TCAST(x) thrust::raw_pointer_cast(x.data())
#define mR3BY3CAST(x) (Real3By3*) thrust::raw_pointer_cast(&x[0])

// ----------------------------------------------------------------------------
// Values
// ----------------------------------------------------------------------------
#define LARGE_NUMBER 99999999
#define SMALL_NUMBER -99999999

// ----------------------------------------------------------------------------
// cutilSafeCall
// CUT_CHECK_ERROR
//
// Legacy CUTIL macros. Currently default to no-ops (TODO)
// ----------------------------------------------------------------------------
#define cudaCheckError()                                                               \
  {                                                                                    \
    cudaError_t e = cudaGetLastError();                                                \
    if (e != cudaSuccess) {                                                            \
      printf("Cuda failure %s:%d: '%s'\n", __FILE__, __LINE__, cudaGetErrorString(e)); \
      exit(0);                                                                         \
    }                                                                                  \
  }

// --------------------------------------------------------------------
// GpuTimer
//
// This utility class encapsulates a simple timer for recording the
// time between a start and stop event.
// --------------------------------------------------------------------
class GpuTimer {
public:
	GpuTimer(cudaStream_t stream = 0) :
			m_stream(stream) {
		cudaEventCreate(&m_start);
		cudaEventCreate(&m_stop);
	}

	~GpuTimer() {
		cudaEventDestroy(m_start);
		cudaEventDestroy(m_stop);
	}

	void Start() {
		cudaEventRecord(m_start, m_stream);
	}
	void Stop() {
		cudaEventRecord(m_stop, m_stream);
	}

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

} // end namespace fsi
} // end namespace chrono
#endif
