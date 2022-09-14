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
// Author: Arman Pazouki
// =============================================================================
//
// Math utilities for the Chrono::FSI module.
// These functions can be invoked either on the CPU (host) or on the GPU (device)
//
// =============================================================================

#ifndef CHFSI_CUSTOM_MATH_H
#define CHFSI_CUSTOM_MATH_H

#include <cuda_runtime.h>
#ifndef __CUDACC__
#include <cmath>
#endif
#include "chrono_fsi/ChConfigFSI.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_math
/// @{

/// Define the real type used in FSI (float or double).
#ifdef CHRONO_FSI_USE_DOUBLE
typedef double Real;
#else
typedef float Real;
#endif

/// Define the unsigned int type used in FSI.
typedef unsigned int uint;

/// Define the unsigned short type used in FSI.
typedef unsigned short ushort;

/// Return the minimum of two single precision numbers.
inline __host__ __device__ float fminf(float a, float b) {
    return a < b ? a : b;
}

/// Return the maximum of two single precision numbers.
inline __host__ __device__ float fmaxf(float a, float b) {
    return a > b ? a : b;
}

/// Return the maximum of two integer numbers.
inline __host__ __device__ int max(int a, int b) {
    return a > b ? a : b;
}

/// Return the minimum of two integer numbers.
inline __host__ __device__ int min(int a, int b) {
    return a < b ? a : b;
}

/// Return the reciprocal square root of a single precision number.
inline __host__ __device__ float rsqrtf(float x) {
    return 1.0f / sqrtf(x);
}

/// Square a float value.
inline __host__ __device__ Real square(Real a) {
    return a * a;
}

/// Cube a float value.
inline __host__ __device__ Real cube(Real a) {
    return a * a * a;
}

/// Quartic a float value.
inline __host__ __device__ Real quartic(Real a) {
    return a * a * a * a;
}

/// Quintic a float value.
inline __host__ __device__ Real quintic(Real a) {
    return a * a * a * a * a;
}

////////////////////////////////////////////////////////////////////////////////
// implementations of basic cuda types
////////////////////////////////////////////////////////////////////////////////

#if defined(__CUDACC_RTC__)
#define __VECTOR_FUNCTIONS_DECL__ __host__ __device__
#else /* !__CUDACC_RTC__ */
#define __VECTOR_FUNCTIONS_DECL__ static __inline__ __host__ __device__
#endif /* __CUDACC_RTC__ */

/// Make a vector with two unsigned integer elements.
__VECTOR_FUNCTIONS_DECL__ uint2 make_uint2(unsigned int x, unsigned int y) {
    uint2 t;
    t.x = x;
    t.y = y;
    return t;
}

/// Make a vector with three unsigned integer elements.
__VECTOR_FUNCTIONS_DECL__ uint3 make_uint3(unsigned int x, unsigned int y, unsigned int z) {
    uint3 t;
    t.x = x;
    t.y = y;
    t.z = z;
    return t;
}

/// Make a vector with four unsigned integer elements.
__VECTOR_FUNCTIONS_DECL__ uint4 make_uint4(unsigned int x, unsigned int y, unsigned int z, unsigned int w) {
    uint4 t;
    t.x = x;
    t.y = y;
    t.z = z;
    t.w = w;
    return t;
}

/// Make a vector with two integer elements.
__VECTOR_FUNCTIONS_DECL__ int2 make_int2(int x, int y) {
    int2 t;
    t.x = x;
    t.y = y;
    return t;
}

/// Make a vector with three integer elements.
__VECTOR_FUNCTIONS_DECL__ int3 make_int3(int x, int y, int z) {
    int3 t;
    t.x = x;
    t.y = y;
    t.z = z;
    return t;
}

/// Make a vector with four integer elements.
__VECTOR_FUNCTIONS_DECL__ int4 make_int4(int x, int y, int z, int w) {
    int4 t;
    t.x = x;
    t.y = y;
    t.z = z;
    t.w = w;
    return t;
}

/// Make a vector with two float elements.
__VECTOR_FUNCTIONS_DECL__ float2 make_float2(float x, float y) {
    float2 t;
    t.x = x;
    t.y = y;
    return t;
}

/// Make a vector with three float elements.
__VECTOR_FUNCTIONS_DECL__ float3 make_float3(float x, float y, float z) {
    float3 t;
    t.x = x;
    t.y = y;
    t.z = z;
    return t;
}

/// Make a vector with four float elements.
__VECTOR_FUNCTIONS_DECL__ float4 make_float4(float x, float y, float z, float w) {
    float4 t;
    t.x = x;
    t.y = y;
    t.z = z;
    t.w = w;
    return t;
}

/// Make a vector with two double elements.
__VECTOR_FUNCTIONS_DECL__ double2 make_double2(double x, double y) {
    double2 t;
    t.x = x;
    t.y = y;
    return t;
}

/// Make a vector with three double elements.
__VECTOR_FUNCTIONS_DECL__ double3 make_double3(double x, double y, double z) {
    double3 t;
    t.x = x;
    t.y = y;
    t.z = z;
    return t;
}

/// Make a vector with four double elements.
__VECTOR_FUNCTIONS_DECL__ double4 make_double4(double x, double y, double z, double w) {
    double4 t;
    t.x = x;
    t.y = y;
    t.z = z;
    t.w = w;
    return t;
}

////////////////////////////////////////////////////////////////////////////////
// constructors
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 make_float2(float s) {
    return make_float2(s, s);
}
inline __host__ __device__ float2 make_float2(float3 a) {
    return make_float2(a.x, a.y);
}
inline __host__ __device__ float2 make_float2(int2 a) {
    return make_float2(float(a.x), float(a.y));
}
inline __host__ __device__ float2 make_float2(uint2 a) {
    return make_float2(float(a.x), float(a.y));
}

inline __host__ __device__ int2 make_int2(int s) {
    return make_int2(s, s);
}
inline __host__ __device__ int2 make_int2(int3 a) {
    return make_int2(a.x, a.y);
}
inline __host__ __device__ int2 make_int2(int4 a) {
    return make_int2(a.x, a.y);
}
inline __host__ __device__ int2 make_int2(uint2 a) {
    return make_int2(int(a.x), int(a.y));
}
inline __host__ __device__ int2 make_int2(float2 a) {
    return make_int2(int(a.x), int(a.y));
}

inline __host__ __device__ uint2 make_uint2(uint s) {
    return make_uint2(s, s);
}
inline __host__ __device__ uint2 make_uint2(uint3 a) {
    return make_uint2(a.x, a.y);
}
inline __host__ __device__ uint2 make_uint2(int2 a) {
    return make_uint2(uint(a.x), uint(a.y));
}

inline __host__ __device__ float3 make_float3(float s) {
    return make_float3(s, s, s);
}
inline __host__ __device__ float3 make_float3(float2 a) {
    return make_float3(a.x, a.y, 0.0f);
}
inline __host__ __device__ float3 make_float3(float2 a, float s) {
    return make_float3(a.x, a.y, s);
}
inline __host__ __device__ float3 make_float3(float4 a) {
    return make_float3(a.x, a.y, a.z);
}
inline __host__ __device__ float3 make_float3(int3 a) {
    return make_float3(float(a.x), float(a.y), float(a.z));
}
inline __host__ __device__ float3 make_float3(uint3 a) {
    return make_float3(float(a.x), float(a.y), float(a.z));
}

inline __host__ __device__ int3 make_int3(int s) {
    return make_int3(s, s, s);
}
inline __host__ __device__ int3 make_int3(int2 a) {
    return make_int3(a.x, a.y, 0);
}
inline __host__ __device__ int3 make_int3(int2 a, int s) {
    return make_int3(a.x, a.y, s);
}
inline __host__ __device__ int3 make_int3(uint3 a) {
    return make_int3(int(a.x), int(a.y), int(a.z));
}
inline __host__ __device__ int3 make_int3(float3 a) {
    return make_int3(int(a.x), int(a.y), int(a.z));
}

inline __host__ __device__ uint3 make_uint3(uint s) {
    return make_uint3(s, s, s);
}
inline __host__ __device__ uint3 make_uint3(uint2 a) {
    return make_uint3(a.x, a.y, 0);
}
inline __host__ __device__ uint3 make_uint3(uint2 a, uint s) {
    return make_uint3(a.x, a.y, s);
}
inline __host__ __device__ uint3 make_uint3(uint4 a) {
    return make_uint3(a.x, a.y, a.z);
}
inline __host__ __device__ uint3 make_uint3(int3 a) {
    return make_uint3(uint(a.x), uint(a.y), uint(a.z));
}

inline __host__ __device__ float4 make_float4(float s) {
    return make_float4(s, s, s, s);
}
inline __host__ __device__ float4 make_float4(float3 a) {
    return make_float4(a.x, a.y, a.z, 0.0f);
}
inline __host__ __device__ float4 make_float4(float3 a, float w) {
    return make_float4(a.x, a.y, a.z, w);
}
inline __host__ __device__ float4 make_float4(int4 a) {
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}
inline __host__ __device__ float4 make_float4(uint4 a) {
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}

inline __host__ __device__ int4 make_int4(int s) {
    return make_int4(s, s, s, s);
}
inline __host__ __device__ int4 make_int4(int3 a) {
    return make_int4(a.x, a.y, a.z, 0);
}
inline __host__ __device__ int4 make_int4(int3 a, int w) {
    return make_int4(a.x, a.y, a.z, w);
}
inline __host__ __device__ int4 make_int4(uint4 a) {
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}
inline __host__ __device__ int4 make_int4(float4 a) {
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}

inline __host__ __device__ uint4 make_uint4(uint s) {
    return make_uint4(s, s, s, s);
}
inline __host__ __device__ uint4 make_uint4(uint3 a) {
    return make_uint4(a.x, a.y, a.z, 0);
}
inline __host__ __device__ uint4 make_uint4(uint3 a, uint w) {
    return make_uint4(a.x, a.y, a.z, w);
}
inline __host__ __device__ uint4 make_uint4(int4 a) {
    return make_uint4(uint(a.x), uint(a.y), uint(a.z), uint(a.w));
}

////////////////////////////////////////////////////////////////////////////////
// negate
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 operator-(float2& a) {
    return make_float2(-a.x, -a.y);
}
inline __host__ __device__ int2 operator-(int2& a) {
    return make_int2(-a.x, -a.y);
}
inline __host__ __device__ float3 operator-(float3& a) {
    return make_float3(-a.x, -a.y, -a.z);
}
inline __host__ __device__ int3 operator-(int3& a) {
    return make_int3(-a.x, -a.y, -a.z);
}
inline __host__ __device__ float4 operator-(float4& a) {
    return make_float4(-a.x, -a.y, -a.z, -a.w);
}
inline __host__ __device__ int4 operator-(int4& a) {
    return make_int4(-a.x, -a.y, -a.z, -a.w);
}

////////////////////////////////////////////////////////////////////////////////
// addition
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 operator+(float2 a, float2 b) {
    return make_float2(a.x + b.x, a.y + b.y);
}
inline __host__ __device__ void operator+=(float2& a, float2 b) {
    a.x += b.x;
    a.y += b.y;
}
inline __host__ __device__ float2 operator+(float2 a, float b) {
    return make_float2(a.x + b, a.y + b);
}
inline __host__ __device__ float2 operator+(float b, float2 a) {
    return make_float2(a.x + b, a.y + b);
}
inline __host__ __device__ void operator+=(float2& a, float b) {
    a.x += b;
    a.y += b;
}

inline __host__ __device__ int2 operator+(int2 a, int2 b) {
    return make_int2(a.x + b.x, a.y + b.y);
}
inline __host__ __device__ void operator+=(int2& a, int2 b) {
    a.x += b.x;
    a.y += b.y;
}
inline __host__ __device__ int2 operator+(int2 a, int b) {
    return make_int2(a.x + b, a.y + b);
}
inline __host__ __device__ int2 operator+(int b, int2 a) {
    return make_int2(a.x + b, a.y + b);
}
inline __host__ __device__ void operator+=(int2& a, int b) {
    a.x += b;
    a.y += b;
}

inline __host__ __device__ uint2 operator+(uint2 a, uint2 b) {
    return make_uint2(a.x + b.x, a.y + b.y);
}
inline __host__ __device__ void operator+=(uint2& a, uint2 b) {
    a.x += b.x;
    a.y += b.y;
}
inline __host__ __device__ uint2 operator+(uint2 a, uint b) {
    return make_uint2(a.x + b, a.y + b);
}
inline __host__ __device__ uint2 operator+(uint b, uint2 a) {
    return make_uint2(a.x + b, a.y + b);
}
inline __host__ __device__ void operator+=(uint2& a, uint b) {
    a.x += b;
    a.y += b;
}

inline __host__ __device__ float3 operator+(float3 a, float3 b) {
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline __host__ __device__ void operator+=(float3& a, float3 b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
inline __host__ __device__ float3 operator+(float3 a, float b) {
    return make_float3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ void operator+=(float3& a, float b) {
    a.x += b;
    a.y += b;
    a.z += b;
}

inline __host__ __device__ int3 operator+(int3 a, int3 b) {
    return make_int3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline __host__ __device__ void operator+=(int3& a, int3 b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
inline __host__ __device__ int3 operator+(int3 a, int b) {
    return make_int3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ void operator+=(int3& a, int b) {
    a.x += b;
    a.y += b;
    a.z += b;
}

inline __host__ __device__ uint3 operator+(uint3 a, uint3 b) {
    return make_uint3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline __host__ __device__ void operator+=(uint3& a, uint3 b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
inline __host__ __device__ uint3 operator+(uint3 a, uint b) {
    return make_uint3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ void operator+=(uint3& a, uint b) {
    a.x += b;
    a.y += b;
    a.z += b;
}

inline __host__ __device__ int3 operator+(int b, int3 a) {
    return make_int3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ uint3 operator+(uint b, uint3 a) {
    return make_uint3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ float3 operator+(float b, float3 a) {
    return make_float3(a.x + b, a.y + b, a.z + b);
}

inline __host__ __device__ float4 operator+(float4 a, float4 b) {
    return make_float4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}
inline __host__ __device__ void operator+=(float4& a, float4 b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
inline __host__ __device__ float4 operator+(float4 a, float b) {
    return make_float4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline __host__ __device__ float4 operator+(float b, float4 a) {
    return make_float4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline __host__ __device__ void operator+=(float4& a, float b) {
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

inline __host__ __device__ int4 operator+(int4 a, int4 b) {
    return make_int4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}
inline __host__ __device__ void operator+=(int4& a, int4 b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
inline __host__ __device__ int4 operator+(int4 a, int b) {
    return make_int4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline __host__ __device__ int4 operator+(int b, int4 a) {
    return make_int4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline __host__ __device__ void operator+=(int4& a, int b) {
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

inline __host__ __device__ uint4 operator+(uint4 a, uint4 b) {
    return make_uint4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}
inline __host__ __device__ void operator+=(uint4& a, uint4 b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
inline __host__ __device__ uint4 operator+(uint4 a, uint b) {
    return make_uint4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline __host__ __device__ uint4 operator+(uint b, uint4 a) {
    return make_uint4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline __host__ __device__ void operator+=(uint4& a, uint b) {
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

////////////////////////////////////////////////////////////////////////////////
// subtract
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 operator-(float2 a, float2 b) {
    return make_float2(a.x - b.x, a.y - b.y);
}
inline __host__ __device__ void operator-=(float2& a, float2 b) {
    a.x -= b.x;
    a.y -= b.y;
}
inline __host__ __device__ float2 operator-(float2 a, float b) {
    return make_float2(a.x - b, a.y - b);
}
inline __host__ __device__ float2 operator-(float b, float2 a) {
    return make_float2(b - a.x, b - a.y);
}
inline __host__ __device__ void operator-=(float2& a, float b) {
    a.x -= b;
    a.y -= b;
}

inline __host__ __device__ int2 operator-(int2 a, int2 b) {
    return make_int2(a.x - b.x, a.y - b.y);
}
inline __host__ __device__ void operator-=(int2& a, int2 b) {
    a.x -= b.x;
    a.y -= b.y;
}
inline __host__ __device__ int2 operator-(int2 a, int b) {
    return make_int2(a.x - b, a.y - b);
}
inline __host__ __device__ int2 operator-(int b, int2 a) {
    return make_int2(b - a.x, b - a.y);
}
inline __host__ __device__ void operator-=(int2& a, int b) {
    a.x -= b;
    a.y -= b;
}

inline __host__ __device__ uint2 operator-(uint2 a, uint2 b) {
    return make_uint2(a.x - b.x, a.y - b.y);
}
inline __host__ __device__ void operator-=(uint2& a, uint2 b) {
    a.x -= b.x;
    a.y -= b.y;
}
inline __host__ __device__ uint2 operator-(uint2 a, uint b) {
    return make_uint2(a.x - b, a.y - b);
}
inline __host__ __device__ uint2 operator-(uint b, uint2 a) {
    return make_uint2(b - a.x, b - a.y);
}
inline __host__ __device__ void operator-=(uint2& a, uint b) {
    a.x -= b;
    a.y -= b;
}

inline __host__ __device__ float3 operator-(float3 a, float3 b) {
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline __host__ __device__ void operator-=(float3& a, float3 b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
inline __host__ __device__ float3 operator-(float3 a, float b) {
    return make_float3(a.x - b, a.y - b, a.z - b);
}
inline __host__ __device__ float3 operator-(float b, float3 a) {
    return make_float3(b - a.x, b - a.y, b - a.z);
}
inline __host__ __device__ void operator-=(float3& a, float b) {
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline __host__ __device__ int3 operator-(int3 a, int3 b) {
    return make_int3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline __host__ __device__ void operator-=(int3& a, int3 b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
inline __host__ __device__ int3 operator-(int3 a, int b) {
    return make_int3(a.x - b, a.y - b, a.z - b);
}
inline __host__ __device__ int3 operator-(int b, int3 a) {
    return make_int3(b - a.x, b - a.y, b - a.z);
}
inline __host__ __device__ void operator-=(int3& a, int b) {
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline __host__ __device__ uint3 operator-(uint3 a, uint3 b) {
    return make_uint3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline __host__ __device__ void operator-=(uint3& a, uint3 b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
inline __host__ __device__ uint3 operator-(uint3 a, uint b) {
    return make_uint3(a.x - b, a.y - b, a.z - b);
}
inline __host__ __device__ uint3 operator-(uint b, uint3 a) {
    return make_uint3(b - a.x, b - a.y, b - a.z);
}
inline __host__ __device__ void operator-=(uint3& a, uint b) {
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline __host__ __device__ float4 operator-(float4 a, float4 b) {
    return make_float4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}
inline __host__ __device__ void operator-=(float4& a, float4 b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
inline __host__ __device__ float4 operator-(float4 a, float b) {
    return make_float4(a.x - b, a.y - b, a.z - b, a.w - b);
}
inline __host__ __device__ void operator-=(float4& a, float b) {
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

inline __host__ __device__ int4 operator-(int4 a, int4 b) {
    return make_int4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}
inline __host__ __device__ void operator-=(int4& a, int4 b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
inline __host__ __device__ int4 operator-(int4 a, int b) {
    return make_int4(a.x - b, a.y - b, a.z - b, a.w - b);
}
inline __host__ __device__ int4 operator-(int b, int4 a) {
    return make_int4(b - a.x, b - a.y, b - a.z, b - a.w);
}
inline __host__ __device__ void operator-=(int4& a, int b) {
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

inline __host__ __device__ uint4 operator-(uint4 a, uint4 b) {
    return make_uint4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}
inline __host__ __device__ void operator-=(uint4& a, uint4 b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
inline __host__ __device__ uint4 operator-(uint4 a, uint b) {
    return make_uint4(a.x - b, a.y - b, a.z - b, a.w - b);
}
inline __host__ __device__ uint4 operator-(uint b, uint4 a) {
    return make_uint4(b - a.x, b - a.y, b - a.z, b - a.w);
}
inline __host__ __device__ void operator-=(uint4& a, uint b) {
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

////////////////////////////////////////////////////////////////////////////////
// multiply
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 operator*(float2 a, float2 b) {
    return make_float2(a.x * b.x, a.y * b.y);
}
inline __host__ __device__ void operator*=(float2& a, float2 b) {
    a.x *= b.x;
    a.y *= b.y;
}
inline __host__ __device__ float2 operator*(float2 a, float b) {
    return make_float2(a.x * b, a.y * b);
}
inline __host__ __device__ float2 operator*(float b, float2 a) {
    return make_float2(b * a.x, b * a.y);
}
inline __host__ __device__ void operator*=(float2& a, float b) {
    a.x *= b;
    a.y *= b;
}

inline __host__ __device__ int2 operator*(int2 a, int2 b) {
    return make_int2(a.x * b.x, a.y * b.y);
}
inline __host__ __device__ void operator*=(int2& a, int2 b) {
    a.x *= b.x;
    a.y *= b.y;
}
inline __host__ __device__ int2 operator*(int2 a, int b) {
    return make_int2(a.x * b, a.y * b);
}
inline __host__ __device__ int2 operator*(int b, int2 a) {
    return make_int2(b * a.x, b * a.y);
}
inline __host__ __device__ void operator*=(int2& a, int b) {
    a.x *= b;
    a.y *= b;
}

inline __host__ __device__ uint2 operator*(uint2 a, uint2 b) {
    return make_uint2(a.x * b.x, a.y * b.y);
}
inline __host__ __device__ void operator*=(uint2& a, uint2 b) {
    a.x *= b.x;
    a.y *= b.y;
}
inline __host__ __device__ uint2 operator*(uint2 a, uint b) {
    return make_uint2(a.x * b, a.y * b);
}
inline __host__ __device__ uint2 operator*(uint b, uint2 a) {
    return make_uint2(b * a.x, b * a.y);
}
inline __host__ __device__ void operator*=(uint2& a, uint b) {
    a.x *= b;
    a.y *= b;
}

inline __host__ __device__ float3 operator*(float3 a, float3 b) {
    return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline __host__ __device__ void operator*=(float3& a, float3 b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
inline __host__ __device__ float3 operator*(float3 a, float b) {
    return make_float3(a.x * b, a.y * b, a.z * b);
}
inline __host__ __device__ float3 operator*(float b, float3 a) {
    return make_float3(b * a.x, b * a.y, b * a.z);
}
inline __host__ __device__ void operator*=(float3& a, float b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline __host__ __device__ int3 operator*(int3 a, int3 b) {
    return make_int3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline __host__ __device__ void operator*=(int3& a, int3 b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
inline __host__ __device__ int3 operator*(int3 a, int b) {
    return make_int3(a.x * b, a.y * b, a.z * b);
}
inline __host__ __device__ int3 operator*(int b, int3 a) {
    return make_int3(b * a.x, b * a.y, b * a.z);
}
inline __host__ __device__ void operator*=(int3& a, int b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline __host__ __device__ uint3 operator*(uint3 a, uint3 b) {
    return make_uint3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline __host__ __device__ void operator*=(uint3& a, uint3 b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
inline __host__ __device__ uint3 operator*(uint3 a, uint b) {
    return make_uint3(a.x * b, a.y * b, a.z * b);
}
inline __host__ __device__ uint3 operator*(uint b, uint3 a) {
    return make_uint3(b * a.x, b * a.y, b * a.z);
}
inline __host__ __device__ void operator*=(uint3& a, uint b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline __host__ __device__ float4 operator*(float4 a, float4 b) {
    return make_float4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}
inline __host__ __device__ void operator*=(float4& a, float4 b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
inline __host__ __device__ float4 operator*(float4 a, float b) {
    return make_float4(a.x * b, a.y * b, a.z * b, a.w * b);
}
inline __host__ __device__ float4 operator*(float b, float4 a) {
    return make_float4(b * a.x, b * a.y, b * a.z, b * a.w);
}
inline __host__ __device__ void operator*=(float4& a, float b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

inline __host__ __device__ int4 operator*(int4 a, int4 b) {
    return make_int4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}
inline __host__ __device__ void operator*=(int4& a, int4 b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
inline __host__ __device__ int4 operator*(int4 a, int b) {
    return make_int4(a.x * b, a.y * b, a.z * b, a.w * b);
}
inline __host__ __device__ int4 operator*(int b, int4 a) {
    return make_int4(b * a.x, b * a.y, b * a.z, b * a.w);
}
inline __host__ __device__ void operator*=(int4& a, int b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

inline __host__ __device__ uint4 operator*(uint4 a, uint4 b) {
    return make_uint4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}
inline __host__ __device__ void operator*=(uint4& a, uint4 b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
inline __host__ __device__ uint4 operator*(uint4 a, uint b) {
    return make_uint4(a.x * b, a.y * b, a.z * b, a.w * b);
}
inline __host__ __device__ uint4 operator*(uint b, uint4 a) {
    return make_uint4(b * a.x, b * a.y, b * a.z, b * a.w);
}
inline __host__ __device__ void operator*=(uint4& a, uint b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

////////////////////////////////////////////////////////////////////////////////
// divide
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 operator/(float2 a, float2 b) {
    return make_float2(a.x / b.x, a.y / b.y);
}
inline __host__ __device__ void operator/=(float2& a, float2 b) {
    a.x /= b.x;
    a.y /= b.y;
}
inline __host__ __device__ float2 operator/(float2 a, float b) {
    return make_float2(a.x / b, a.y / b);
}
inline __host__ __device__ void operator/=(float2& a, float b) {
    a.x /= b;
    a.y /= b;
}
inline __host__ __device__ float2 operator/(float b, float2 a) {
    return make_float2(b / a.x, b / a.y);
}

inline __host__ __device__ float3 operator/(float3 a, float3 b) {
    return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
}
inline __host__ __device__ void operator/=(float3& a, float3 b) {
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
}
inline __host__ __device__ float3 operator/(float3 a, float b) {
    return make_float3(a.x / b, a.y / b, a.z / b);
}
inline __host__ __device__ void operator/=(float3& a, float b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
}
inline __host__ __device__ float3 operator/(float b, float3 a) {
    return make_float3(b / a.x, b / a.y, b / a.z);
}

inline __host__ __device__ float4 operator/(float4 a, float4 b) {
    return make_float4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}
inline __host__ __device__ void operator/=(float4& a, float4 b) {
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
    a.w /= b.w;
}
inline __host__ __device__ float4 operator/(float4 a, float b) {
    return make_float4(a.x / b, a.y / b, a.z / b, a.w / b);
}
inline __host__ __device__ void operator/=(float4& a, float b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
    a.w /= b;
}
inline __host__ __device__ float4 operator/(float b, float4 a) {
    return make_float4(b / a.x, b / a.y, b / a.z, b / a.w);
}

////////////////////////////////////////////////////////////////////////////////
// min
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 fminf(float2 a, float2 b) {
    return make_float2(fminf(a.x, b.x), fminf(a.y, b.y));
}
inline __host__ __device__ float3 fminf(float3 a, float3 b) {
    return make_float3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}
inline __host__ __device__ float4 fminf(float4 a, float4 b) {
    return make_float4(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z), fminf(a.w, b.w));
}

inline __host__ __device__ int2 min(int2 a, int2 b) {
    return make_int2(min(a.x, b.x), min(a.y, b.y));
}
inline __host__ __device__ int3 min(int3 a, int3 b) {
    return make_int3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z));
}
inline __host__ __device__ int4 min(int4 a, int4 b) {
    return make_int4(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z), min(a.w, b.w));
}

inline __host__ __device__ uint2 min(uint2 a, uint2 b) {
    return make_uint2(min(a.x, b.x), min(a.y, b.y));
}
inline __host__ __device__ uint3 min(uint3 a, uint3 b) {
    return make_uint3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z));
}
inline __host__ __device__ uint4 min(uint4 a, uint4 b) {
    return make_uint4(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z), min(a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// max
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 fmaxf(float2 a, float2 b) {
    return make_float2(fmaxf(a.x, b.x), fmaxf(a.y, b.y));
}
inline __host__ __device__ float3 fmaxf(float3 a, float3 b) {
    return make_float3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}
inline __host__ __device__ float4 fmaxf(float4 a, float4 b) {
    return make_float4(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z), fmaxf(a.w, b.w));
}

inline __host__ __device__ int2 max(int2 a, int2 b) {
    return make_int2(max(a.x, b.x), max(a.y, b.y));
}
inline __host__ __device__ int3 max(int3 a, int3 b) {
    return make_int3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z));
}
inline __host__ __device__ int4 max(int4 a, int4 b) {
    return make_int4(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z), max(a.w, b.w));
}

inline __host__ __device__ uint2 max(uint2 a, uint2 b) {
    return make_uint2(max(a.x, b.x), max(a.y, b.y));
}
inline __host__ __device__ uint3 max(uint3 a, uint3 b) {
    return make_uint3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z));
}
inline __host__ __device__ uint4 max(uint4 a, uint4 b) {
    return make_uint4(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z), max(a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// lerp
// - linear interpolation between a and b, based on value t in [0, 1] range
////////////////////////////////////////////////////////////////////////////////

inline __device__ __host__ float lerp(float a, float b, float t) {
    return a + t * (b - a);
}
inline __device__ __host__ float2 lerp(float2 a, float2 b, float t) {
    return a + t * (b - a);
}
inline __device__ __host__ float3 lerp(float3 a, float3 b, float t) {
    return a + t * (b - a);
}
inline __device__ __host__ float4 lerp(float4 a, float4 b, float t) {
    return a + t * (b - a);
}

////////////////////////////////////////////////////////////////////////////////
// dot product
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float dot(float2 a, float2 b) {
    return a.x * b.x + a.y * b.y;
}
inline __host__ __device__ float dot(float3 a, float3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __host__ __device__ float dot(float4 a, float4 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline __host__ __device__ int dot(int2 a, int2 b) {
    return a.x * b.x + a.y * b.y;
}
inline __host__ __device__ int dot(int3 a, int3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __host__ __device__ int dot(int4 a, int4 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline __host__ __device__ uint dot(uint2 a, uint2 b) {
    return a.x * b.x + a.y * b.y;
}
inline __host__ __device__ uint dot(uint3 a, uint3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __host__ __device__ uint dot(uint4 a, uint4 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

////////////////////////////////////////////////////////////////////////////////
// length
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float length(float2 v) {
    return sqrtf(dot(v, v));
}
inline __host__ __device__ float length(float3 v) {
    return sqrtf(dot(v, v));
}
inline __host__ __device__ float length(float4 v) {
    return sqrtf(dot(v, v));
}

////////////////////////////////////////////////////////////////////////////////
// normalize
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 normalize(float2 v) {
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}
inline __host__ __device__ float3 normalize(float3 v) {
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}
inline __host__ __device__ float4 normalize(float4 v) {
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}

////////////////////////////////////////////////////////////////////////////////
// cross product
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float3 cross(float3 a, float3 b) {
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

////////////////////////////////////////////////////////////////////////////////
// function definitions for the Real type
////////////////////////////////////////////////////////////////////////////////

struct Real2 {
    Real x;
    Real y;
};
struct Real3 {
    Real x;
    Real y;
    Real z;
};
struct Real4 {
    Real x;
    Real y;
    Real z;
    Real w;
};

__host__ __device__ inline Real rminr(Real a, Real b) {
    return a < b ? a : b;
}

__host__ __device__ inline Real rmaxr(Real a, Real b) {
    return a > b ? a : b;
}

__host__ __device__ inline Real rsqrtr(Real x) {
    return 1.0 / sqrt(x);
}

__host__ __device__ inline Real2 make_Real2(Real b, Real c)  ///
{
    Real2 a;
    a.x = b;
    a.y = c;
    return a;
}
__host__ __device__ inline Real2 make_Real2(Real s) {
    return make_Real2(s, s);
}
__host__ __device__ inline Real2 make_Real2(Real3 a) {
    return make_Real2(a.x, a.y);
}
__host__ __device__ inline Real2 make_Real2(int2 a) {
    return make_Real2(Real(a.x), Real(a.y));
}
__host__ __device__ inline Real2 make_Real2(uint2 a) {
    return make_Real2(Real(a.x), Real(a.y));
}

__host__ __device__ inline int2 make_int2(Real2 a) {
    return make_int2(int(a.x), int(a.y));
}

__host__ __device__ inline Real3 make_Real3(Real a, Real b, Real c)  ///
{
    Real3 d;
    d.x = a;
    d.y = b;
    d.z = c;
    return d;
}
__host__ __device__ inline Real3 make_Real3(Real s) {
    return make_Real3(s, s, s);
}
__host__ __device__ inline Real3 make_Real3(Real2 a) {
    return make_Real3(a.x, a.y, 0.0);
}
__host__ __device__ inline Real3 make_Real3(Real2 a, Real s) {
    return make_Real3(a.x, a.y, s);
}
__host__ __device__ inline Real3 make_Real3(Real4 a) {
    return make_Real3(a.x, a.y, a.z);
}
__host__ __device__ inline Real3 make_Real3(int3 a) {
    return make_Real3(Real(a.x), Real(a.y), Real(a.z));
}
__host__ __device__ inline Real3 make_Real3(uint3 a) {
    return make_Real3(Real(a.x), Real(a.y), Real(a.z));
}

__host__ __device__ inline int3 make_int3(Real3 a) {
    return make_int3(int(a.x), int(a.y), int(a.z));
}
__host__ __device__ inline Real3 make_Real3(Real3 a) {
    return make_Real3(a.x, a.y, a.z);
}

__host__ __device__ inline Real4 make_Real4(Real a, Real b, Real c, Real d)  ///
{
    Real4 e;
    e.x = a;
    e.y = b;
    e.z = c;
    e.w = d;
    return e;
}
__host__ __device__ inline Real4 make_Real4(Real s) {
    return make_Real4(s, s, s, s);
}
__host__ __device__ inline Real4 make_Real4(Real3 a) {
    return make_Real4(a.x, a.y, a.z, 0.0);
}
__host__ __device__ inline Real4 make_Real4(Real3 a, Real w) {
    return make_Real4(a.x, a.y, a.z, w);
}
__host__ __device__ inline Real4 make_Real4(Real4 a) {
    return make_Real4(a.x, a.y, a.z, a.w);
}
__host__ __device__ inline Real4 make_Real4(int4 a) {
    return make_Real4(Real(a.x), Real(a.y), Real(a.z), Real(a.w));
}
__host__ __device__ inline Real4 make_Real4(uint4 a) {
    return make_Real4(Real(a.x), Real(a.y), Real(a.z), Real(a.w));
}

__host__ __device__ inline int4 make_int4(Real4 a) {
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}

__host__ __device__ inline Real2 operator-(Real2& a) {
    return make_Real2(-a.x, -a.y);
}

__host__ __device__ inline Real3 operator-(Real3& a) {
    return make_Real3(-a.x, -a.y, -a.z);
}

__host__ __device__ inline Real4 operator-(Real4& a) {
    return make_Real4(-a.x, -a.y, -a.z, -a.w);
}

__host__ __device__ inline Real2 operator+(Real2 a, Real2 b) {
    return make_Real2(a.x + b.x, a.y + b.y);
}
__host__ __device__ inline void operator+=(Real2& a, Real2 b) {
    a.x += b.x;
    a.y += b.y;
}
__host__ __device__ inline Real2 operator+(Real2 a, Real b) {
    return make_Real2(a.x + b, a.y + b);
}
__host__ __device__ inline Real2 operator+(Real b, Real2 a) {
    return make_Real2(a.x + b, a.y + b);
}
__host__ __device__ inline void operator+=(Real2& a, Real b) {
    a.x += b;
    a.y += b;
}

__host__ __device__ inline Real3 operator+(Real3 a, Real3 b) {
    return make_Real3(a.x + b.x, a.y + b.y, a.z + b.z);
}
__host__ __device__ inline void operator+=(Real3& a, Real3 b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
__host__ __device__ inline Real3 operator+(Real3 a, Real b) {
    return make_Real3(a.x + b, a.y + b, a.z + b);
}
__host__ __device__ inline void operator+=(Real3& a, Real b) {
    a.x += b;
    a.y += b;
    a.z += b;
}

__host__ __device__ inline Real3 operator+(Real b, Real3 a) {
    return make_Real3(a.x + b, a.y + b, a.z + b);
}

__host__ __device__ inline Real4 operator+(Real4 a, Real4 b) {
    return make_Real4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}
__host__ __device__ inline void operator+=(Real4& a, Real4 b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
__host__ __device__ inline Real4 operator+(Real4 a, Real b) {
    return make_Real4(a.x + b, a.y + b, a.z + b, a.w + b);
}
__host__ __device__ inline Real4 operator+(Real b, Real4 a) {
    return make_Real4(a.x + b, a.y + b, a.z + b, a.w + b);
}
__host__ __device__ inline void operator+=(Real4& a, Real b) {
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

__host__ __device__ inline Real2 operator-(Real2 a, Real2 b) {
    return make_Real2(a.x - b.x, a.y - b.y);
}
__host__ __device__ inline void operator-=(Real2& a, Real2 b) {
    a.x -= b.x;
    a.y -= b.y;
}
__host__ __device__ inline Real2 operator-(Real2 a, Real b) {
    return make_Real2(a.x - b, a.y - b);
}
__host__ __device__ inline Real2 operator-(Real b, Real2 a) {
    return make_Real2(b - a.x, b - a.y);
}
__host__ __device__ inline void operator-=(Real2& a, Real b) {
    a.x -= b;
    a.y -= b;
}

__host__ __device__ inline Real3 operator-(Real3 a, Real3 b) {
    return make_Real3(a.x - b.x, a.y - b.y, a.z - b.z);
}
__host__ __device__ inline void operator-=(Real3& a, Real3 b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
__host__ __device__ inline Real3 operator-(Real3 a, Real b) {
    return make_Real3(a.x - b, a.y - b, a.z - b);
}
__host__ __device__ inline Real3 operator-(Real b, Real3 a) {
    return make_Real3(b - a.x, b - a.y, b - a.z);
}
__host__ __device__ inline void operator-=(Real3& a, Real b) {
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

__host__ __device__ inline Real4 operator-(Real4 a, Real4 b) {
    return make_Real4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}
__host__ __device__ inline void operator-=(Real4& a, Real4 b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
__host__ __device__ inline Real4 operator-(Real4 a, Real b) {
    return make_Real4(a.x - b, a.y - b, a.z - b, a.w - b);
}
__host__ __device__ inline void operator-=(Real4& a, Real b) {
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

__host__ __device__ inline Real2 operator*(Real2 a, Real2 b) {
    return make_Real2(a.x * b.x, a.y * b.y);
}
__host__ __device__ inline void operator*=(Real2& a, Real2 b) {
    a.x *= b.x;
    a.y *= b.y;
}
__host__ __device__ inline Real2 operator*(Real2 a, Real b) {
    return make_Real2(a.x * b, a.y * b);
}
__host__ __device__ inline Real2 operator*(Real b, Real2 a) {
    return make_Real2(b * a.x, b * a.y);
}
__host__ __device__ inline void operator*=(Real2& a, Real b) {
    a.x *= b;
    a.y *= b;
}

__host__ __device__ inline Real3 operator*(Real3 a, Real3 b) {
    return make_Real3(a.x * b.x, a.y * b.y, a.z * b.z);
}
__host__ __device__ inline void operator*=(Real3& a, Real3 b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
__host__ __device__ inline Real3 operator*(Real3 a, Real b) {
    return make_Real3(a.x * b, a.y * b, a.z * b);
}
__host__ __device__ inline Real3 operator*(Real b, Real3 a) {
    return make_Real3(b * a.x, b * a.y, b * a.z);
}
__host__ __device__ inline void operator*=(Real3& a, Real b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

__host__ __device__ inline Real4 operator*(Real4 a, Real4 b) {
    return make_Real4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}
__host__ __device__ inline void operator*=(Real4& a, Real4 b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
__host__ __device__ inline Real4 operator*(Real4 a, Real b) {
    return make_Real4(a.x * b, a.y * b, a.z * b, a.w * b);
}
__host__ __device__ inline Real4 operator*(Real b, Real4 a) {
    return make_Real4(b * a.x, b * a.y, b * a.z, b * a.w);
}
__host__ __device__ inline void operator*=(Real4& a, Real b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

__host__ __device__ inline Real2 operator/(Real2 a, Real2 b) {
    return make_Real2(a.x / b.x, a.y / b.y);
}
__host__ __device__ inline void operator/=(Real2& a, Real2 b) {
    a.x /= b.x;
    a.y /= b.y;
}
__host__ __device__ inline Real2 operator/(Real2 a, Real b) {
    return make_Real2(a.x / b, a.y / b);
}
__host__ __device__ inline void operator/=(Real2& a, Real b) {
    a.x /= b;
    a.y /= b;
}
__host__ __device__ inline Real2 operator/(Real b, Real2 a) {
    return make_Real2(b / a.x, b / a.y);
}

__host__ __device__ inline Real3 operator/(Real3 a, Real3 b) {
    return make_Real3(a.x / b.x, a.y / b.y, a.z / b.z);
}
__host__ __device__ inline void operator/=(Real3& a, Real3 b) {
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
}
__host__ __device__ inline Real3 operator/(Real3 a, Real b) {
    return make_Real3(a.x / b, a.y / b, a.z / b);
}
__host__ __device__ inline void operator/=(Real3& a, Real b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
}
__host__ __device__ inline Real3 operator/(Real b, Real3 a) {
    return make_Real3(b / a.x, b / a.y, b / a.z);
}

__host__ __device__ inline Real4 operator/(Real4 a, Real4 b) {
    return make_Real4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}
__host__ __device__ inline void operator/=(Real4& a, Real4 b) {
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
    a.w /= b.w;
}
__host__ __device__ inline Real4 operator/(Real4 a, Real b) {
    return make_Real4(a.x / b, a.y / b, a.z / b, a.w / b);
}
__host__ __device__ inline void operator/=(Real4& a, Real b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
    a.w /= b;
}
__host__ __device__ inline Real4 operator/(Real b, Real4 a) {
    return make_Real4(b / a.x, b / a.y, b / a.z, b / a.w);
}

__host__ __device__ inline Real2 rminr(Real2 a, Real2 b) {
    return make_Real2(rminr(a.x, b.x), rminr(a.y, b.y));
}
__host__ __device__ inline Real3 rminr(Real3 a, Real3 b) {
    return make_Real3(rminr(a.x, b.x), rminr(a.y, b.y), rminr(a.z, b.z));
}
__host__ __device__ inline Real4 rminr(Real4 a, Real4 b) {
    return make_Real4(rminr(a.x, b.x), rminr(a.y, b.y), rminr(a.z, b.z), rminr(a.w, b.w));
}

__host__ __device__ inline Real2 rmaxr(Real2 a, Real2 b) {
    return make_Real2(rmaxr(a.x, b.x), rmaxr(a.y, b.y));
}
__host__ __device__ inline Real3 rmaxr(Real3 a, Real3 b) {
    return make_Real3(rmaxr(a.x, b.x), rmaxr(a.y, b.y), rmaxr(a.z, b.z));
}
__host__ __device__ inline Real4 rmaxr(Real4 a, Real4 b) {
    return make_Real4(rmaxr(a.x, b.x), rmaxr(a.y, b.y), rmaxr(a.z, b.z), rmaxr(a.w, b.w));
}

#if DOUBLEPRECISION
__host__ __device__ inline Real lerp(Real a, Real b, Real t) {
    return a + t * (b - a);
}
#endif

__host__ __device__ inline Real2 lerp(Real2 a, Real2 b, Real t) {
    return a + t * (b - a);
}
__host__ __device__ inline Real3 lerp(Real3 a, Real3 b, Real t) {
    return a + t * (b - a);
}
__host__ __device__ inline Real4 lerp(Real4 a, Real4 b, Real t) {
    return a + t * (b - a);
}

__host__ __device__ inline Real dot(Real2 a, Real2 b) {
    return a.x * b.x + a.y * b.y;
}
__host__ __device__ inline Real dot(Real3 a, Real3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
__host__ __device__ inline Real dot(Real4 a, Real4 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

__host__ __device__ inline Real length(Real v) {
    return abs(v);
}
__host__ __device__ inline Real length(Real2 v) {
    return sqrt(dot(v, v));
}
__host__ __device__ inline Real length(Real3 v) {
    return sqrt(dot(v, v));
}
__host__ __device__ inline Real length(Real4 v) {
    return sqrt(dot(v, v));
}

__host__ __device__ inline Real2 normalize(Real2 v) {
    Real invLen = rsqrtr(dot(v, v));
    return v * invLen;
}
__host__ __device__ inline Real3 normalize(Real3 v) {
    Real invLen = rsqrtr(dot(v, v));
    return v * invLen;
}
__host__ __device__ inline Real4 normalize(Real4 v) {
    Real invLen = rsqrtr(dot(v, v));
    return v * invLen;
}

__host__ __device__ inline Real3 cross(Real3 a, Real3 b) {
    return make_Real3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

__host__ __device__ inline Real sgn(Real a) {
    return a > 0 ? +1.0 : -1.0;
}

__host__ __device__ inline Real3 sgn(Real3 a) {
    return make_Real3(sgn(a.x), sgn(a.y), sgn(a.z));
}

__host__ __device__ inline Real4 Cables_ShapeFunctions(Real l, Real xi) {
        Real N1 = 1 - 3 * square(xi) + 2 * cube(xi);
        Real N2 = l * (xi - 2 * square(xi) + cube(xi));
    	Real N3 = 3 * square(xi) - 2 * cube(xi);
        Real N4 = l * (-square(xi) + cube(xi));

    return make_Real4(N1, N2, N3, N4);
}

__host__ __device__ inline Real4 Cables_ShapeFunctionsDerivatives(Real l, Real xi) {
    	Real N1 = (6 * square(xi) - 6 * xi)/l;
        Real N2 = 1.0 - 4.0 * xi + 3 * square(xi);
    	Real N3 = -(6.0 * square(xi) - 6 * xi) / l;
        Real N4 = -2.0 * xi + 3 * square(xi);

    return make_Real4(N1, N2, N3, N4);
}

__host__ __device__ inline Real4 Shells_ShapeFunctions(Real x, Real y) {
    Real NA = 0.25 * (1.0 - x) * (1.0 - y);
    Real NB = 0.25 * (1.0 + x) * (1.0 - y);
    Real NC = 0.25 * (1.0 + x) * (1.0 + y);
    Real ND = 0.25 * (1.0 - x) * (1.0 + y);
    return make_Real4(NA, NB, NC, ND);
}

__host__ __device__ inline Real3 user_BC_U(Real3 Pos) {
    Real3 vel = make_Real3(0.0, 0.0, 0.0);
    //   User define fucntion for U goes here
    if (Pos.z >= 1.025 && Pos.x <= 0.5 && Pos.x >= -0.5) {
        vel = make_Real3(1, 0, 0);
    }

    return vel;
}

/// @} fsi_math

}  // end namespace fsi
}  // end namespace chrono

#endif
