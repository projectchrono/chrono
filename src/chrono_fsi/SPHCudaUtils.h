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
#include "custom_cutil_math.h"

// ----------------------------------------------------------------------------
// Short-hand notation
// ----------------------------------------------------------------------------
#define F2  make_float2
#define F3  make_float3
#define F4  make_float4
#define R2  make_real2
#define R3  make_real3
#define R4  make_real4

#define I2  make_int2
#define I3  make_int3
#define I4  int4

#define U3  make_uint3

#define F1CAST(x) (float*)thrust::raw_pointer_cast(&x[0])
#define D1CAST(x) (double*)thrust::raw_pointer_cast(&x[0])

#define I1CAST(x) (int*)thrust::raw_pointer_cast(&x[0])
#define I2CAST(x) (int2*)thrust::raw_pointer_cast(&x[0])
#define U1CAST(x) (uint*)thrust::raw_pointer_cast(&x[0])
#define R1CAST(x) (real_*)thrust::raw_pointer_cast(&x[0])
#define R3CAST(x) (real3*)thrust::raw_pointer_cast(&x[0])
#define R4CAST(x) (real4*)thrust::raw_pointer_cast(&x[0])
#define TCAST(x) thrust::raw_pointer_cast(x.data())
#define R3BY3CAST(x) (real3By3*)thrust::raw_pointer_cast(&x[0])

// ----------------------------------------------------------------------------
// Values
// ----------------------------------------------------------------------------
#define LARGE_NUMBER 99999999
#define SMALL_NUMBER -99999999
// ----------------------------------------------------------------------------
// MULT    multiplication of signed integers
// UMULT   multiplication of unsigned integers
//
// These macros are architecture-dependent: on architectures prior to Fermi
// it uses the 24-bit intrinsics; on Fermi and above (32-bit) simply use the
// native multiplication.
// ----------------------------------------------------------------------------
#if __CUDA_ARCH__ >= 200
#define MULT(X, Y)  ((X) * (Y))
#define UMULT(X, Y) ((X) * (Y))
#else
#define MULT(X, Y)  __mul24((X), (Y))
#define UMULT(X, Y) __umul24((X), (Y))
#endif

//#define DOUBLEPRECISION true
//#if DOUBLEPRECISION
//#define FLOAT double
//#else
//#define FLOAT real_
//#endif

// ----------------------------------------------------------------------------
// cutilSafeCall
// CUT_CHECK_ERROR
//
// Legacy CUTIL macros. Currently default to no-ops (TODO)
// ----------------------------------------------------------------------------
#define cutilSafeCall(x)  x
#define CUT_CHECK_ERROR(x) x



// --------------------------------------------------------------------
// GpuTimer
//
// This utility class encapsulates a simple timer for recording the
// time between a start and stop event.
// --------------------------------------------------------------------
class GpuTimer
{
public:
	GpuTimer(cudaStream_t stream = 0) : m_stream(stream) {
		cudaEventCreate(&m_start);
		cudaEventCreate(&m_stop);
	}

	~GpuTimer() {
		cudaEventDestroy(m_start);
		cudaEventDestroy(m_stop);
	}

	void Start() {cudaEventRecord(m_start, m_stream);}
	void Stop()  {cudaEventRecord(m_stop,  m_stream);}

	float Elapsed() {
		float elapsed;
		cudaEventSynchronize(m_stop);
		cudaEventElapsedTime(&elapsed, m_start, m_stop);
		return elapsed;
	}

private:
	cudaStream_t m_stream;
	cudaEvent_t  m_start;
	cudaEvent_t  m_stop;
};



#endif
