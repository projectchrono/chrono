/*
 * Old classic cutil_math customized for the type Real, which can be
 * float or double
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

#ifndef CUSTOM_CUTIL_MATH_H
#define CUSTOM_CUTIL_MATH_H
////////Define Real, either float or double
#include <cuda_runtime.h>

typedef unsigned int uint;
typedef unsigned short ushort;

#define DOUBLEPRECISION 1
#if DOUBLEPRECISION
typedef double Real;
#else
typedef float Real;
#endif

// decide if you want to include helper math stuff, i.e. float uint int stuff, by changing the following def
#define INCLUDECUTILMATHSTUFF 1
#if INCLUDECUTILMATHSTUFF
#ifndef __CUDACC__
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
// host implementations of CUDA functions
////////////////////////////////////////////////////////////////////////////////

inline float fminf(float a, float b) {
  return a < b ? a : b;
}

inline float fmaxf(float a, float b) {
  return a > b ? a : b;
}

inline int max(int a, int b) {
  return a > b ? a : b;
}

inline int min(int a, int b) {
  return a < b ? a : b;
}

inline float rsqrtf(float x) {
  return 1.0f / sqrtf(x);
}
#endif

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
// clamp
// - clamp the value v to be in the range [a, b]
////////////////////////////////////////////////////////////////////////////////
//
// inline __device__ __host__ float clamp(float f, float a, float b)
//{
//    return fmaxf(a, fminf(f, b));
//}
// inline __device__ __host__ int clamp(int f, int a, int b)
//{
//    return max(a, min(f, b));
//}
// inline __device__ __host__ uint clamp(uint f, uint a, uint b)
//{
//    return max(a, min(f, b));
//}
//
// inline __device__ __host__ float2 clamp(float2 v, float a, float b)
//{
//    return make_float2(clamp(v.x, a, b), clamp(v.y, a, b));
//}
// inline __device__ __host__ float2 clamp(float2 v, float2 a, float2 b)
//{
//    return make_float2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
//}
// inline __device__ __host__ float3 clamp(float3 v, float a, float b)
//{
//    return make_float3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
//}
// inline __device__ __host__ float3 clamp(float3 v, float3 a, float3 b)
//{
//    return make_float3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
//}
// inline __device__ __host__ float4 clamp(float4 v, float a, float b)
//{
//    return make_float4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
//}
// inline __device__ __host__ float4 clamp(float4 v, float4 a, float4 b)
//{
//    return make_float4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
//}
//
// inline __device__ __host__ int2 clamp(int2 v, int a, int b)
//{
//    return make_int2(clamp(v.x, a, b), clamp(v.y, a, b));
//}
// inline __device__ __host__ int2 clamp(int2 v, int2 a, int2 b)
//{
//    return make_int2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
//}
// inline __device__ __host__ int3 clamp(int3 v, int a, int b)
//{
//    return make_int3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
//}
// inline __device__ __host__ int3 clamp(int3 v, int3 a, int3 b)
//{
//    return make_int3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
//}
// inline __device__ __host__ int4 clamp(int4 v, int a, int b)
//{
//    return make_int4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
//}
// inline __device__ __host__ int4 clamp(int4 v, int4 a, int4 b)
//{
//    return make_int4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
//}
//
// inline __device__ __host__ uint2 clamp(uint2 v, uint a, uint b)
//{
//    return make_uint2(clamp(v.x, a, b), clamp(v.y, a, b));
//}
// inline __device__ __host__ uint2 clamp(uint2 v, uint2 a, uint2 b)
//{
//    return make_uint2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
//}
// inline __device__ __host__ uint3 clamp(uint3 v, uint a, uint b)
//{
//    return make_uint3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
//}
// inline __device__ __host__ uint3 clamp(uint3 v, uint3 a, uint3 b)
//{
//    return make_uint3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
//}
// inline __device__ __host__ uint4 clamp(uint4 v, uint a, uint b)
//{
//    return make_uint4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
//}
// inline __device__ __host__ uint4 clamp(uint4 v, uint4 a, uint4 b)
//{
//    return make_uint4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
//}

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
// floor
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 floorf(float2 v) {
  return make_float2(floorf(v.x), floorf(v.y));
}
inline __host__ __device__ float3 floorf(float3 v) {
  return make_float3(floorf(v.x), floorf(v.y), floorf(v.z));
}
inline __host__ __device__ float4 floorf(float4 v) {
  return make_float4(floorf(v.x), floorf(v.y), floorf(v.z), floorf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// frac - returns the fractional portion of a scalar or each vector component
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float fracf(float v) {
  return v - floorf(v);
}
inline __host__ __device__ float2 fracf(float2 v) {
  return make_float2(fracf(v.x), fracf(v.y));
}
inline __host__ __device__ float3 fracf(float3 v) {
  return make_float3(fracf(v.x), fracf(v.y), fracf(v.z));
}
inline __host__ __device__ float4 fracf(float4 v) {
  return make_float4(fracf(v.x), fracf(v.y), fracf(v.z), fracf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// fmod
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 fmodf(float2 a, float2 b) {
  return make_float2(fmodf(a.x, b.x), fmodf(a.y, b.y));
}
inline __host__ __device__ float3 fmodf(float3 a, float3 b) {
  return make_float3(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z));
}
inline __host__ __device__ float4 fmodf(float4 a, float4 b) {
  return make_float4(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z), fmodf(a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// absolute value
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float2 fabs(float2 v) {
  return make_float2(fabs(v.x), fabs(v.y));
}
inline __host__ __device__ float3 fabs(float3 v) {
  return make_float3(fabs(v.x), fabs(v.y), fabs(v.z));
}
inline __host__ __device__ float4 fabs(float4 v) {
  return make_float4(fabs(v.x), fabs(v.y), fabs(v.z), fabs(v.w));
}

inline __host__ __device__ int2 abs(int2 v) {
  return make_int2(abs(v.x), abs(v.y));
}
inline __host__ __device__ int3 abs(int3 v) {
  return make_int3(abs(v.x), abs(v.y), abs(v.z));
}
inline __host__ __device__ int4 abs(int4 v) {
  return make_int4(abs(v.x), abs(v.y), abs(v.z), abs(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// reflect
// - returns reflection of incident ray I around surface normal N
// - N should be normalized, reflected vector's length is equal to length of I
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float3 reflect(float3 i, float3 n) {
  return i - 2.0f * n * dot(n, i);
}

////////////////////////////////////////////////////////////////////////////////
// cross product
////////////////////////////////////////////////////////////////////////////////

inline __host__ __device__ float3 cross(float3 a, float3 b) {
  return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

////////////////////////////////////////////////////////////////////////////////
// smoothstep
// - returns 0 if x < a
// - returns 1 if x > b
// - otherwise returns smooth interpolation between 0 and 1 based on x
////////////////////////////////////////////////////////////////////////////////

// inline __device__ __host__ float smoothstep(float a, float b, float x)
//{
//    float y = clamp((x - a) / (b - a), 0.0f, 1.0f);
//    return (y*y*(3.0f - (2.0f*y)));
//}
// inline __device__ __host__ float2 smoothstep(float2 a, float2 b, float2 x)
//{
//    float2 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
//    return (y*y*(make_float2(3.0f) - (make_float2(2.0f)*y)));
//}
// inline __device__ __host__ float3 smoothstep(float3 a, float3 b, float3 x)
//{
//    float3 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
//    return (y*y*(make_float3(3.0f) - (make_float3(2.0f)*y)));
//}
// inline __device__ __host__ float4 smoothstep(float4 a, float4 b, float4 x)
//{
//    float4 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
//    return (y*y*(make_float4(3.0f) - (make_float4(2.0f)*y)));
//}
#endif
//
//// end of helper_math stuff. start of my own stuff

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

//#if DOUBLEPRECISION
//__host__ __device__ inline Real clamp(Real f, Real a, Real b)
//{
//    return rmaxr(a, rminr(f, b));
//}
//#endif
//__host__ __device__ inline Real2 clamp(Real2 v, Real a, Real b)
//{
//    return make_Real2(clamp(v.x, a, b), clamp(v.y, a, b));
//}
//__host__ __device__ inline Real2 clamp(Real2 v, Real2 a, Real2 b)
//{
//    return make_Real2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
//}
//__host__ __device__ inline Real3 clamp(Real3 v, Real a, Real b)
//{
//    return make_Real3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
//}
//__host__ __device__ inline Real3 clamp(Real3 v, Real3 a, Real3 b)
//{
//    return make_Real3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
//}
//__host__ __device__ inline Real4 clamp(Real4 v, Real a, Real b)
//{
//    return make_Real4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
//}
//__host__ __device__ inline Real4 clamp(Real4 v, Real4 a, Real4 b)
//{
//    return make_Real4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
//}

__host__ __device__ inline Real dot(Real2 a, Real2 b) {
  return a.x * b.x + a.y * b.y;
}
__host__ __device__ inline Real dot(Real3 a, Real3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
__host__ __device__ inline Real dot(Real4 a, Real4 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
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

__host__ __device__ inline Real2 floor(Real2 v) {
  return make_Real2(floor(v.x), floor(v.y));
}
__host__ __device__ inline Real3 floor(Real3 v) {
  return make_Real3(floor(v.x), floor(v.y), floor(v.z));
}
__host__ __device__ inline Real4 floor(Real4 v) {
  return make_Real4(floor(v.x), floor(v.y), floor(v.z), floor(v.w));
}

__host__ __device__ inline Real fracr(Real v) {
  return v - floor(v);
}
__host__ __device__ inline Real2 fracr(Real2 v) {
  return make_Real2(fracr(v.x), fracr(v.y));
}
__host__ __device__ inline Real3 fracr(Real3 v) {
  return make_Real3(fracr(v.x), fracr(v.y), fracr(v.z));
}
__host__ __device__ inline Real4 fracr(Real4 v) {
  return make_Real4(fracr(v.x), fracr(v.y), fracr(v.z), fracr(v.w));
}

__host__ __device__ inline Real rmodr(Real a, Real b) {
  return fmod(a, b);
}
__host__ __device__ inline Real2 rmodr(Real2 a, Real2 b) {
  return make_Real2(rmodr(a.x, b.x), rmodr(a.y, b.y));
}
__host__ __device__ inline Real3 rmodr(Real3 a, Real3 b) {
  return make_Real3(rmodr(a.x, b.x), rmodr(a.y, b.y), rmodr(a.z, b.z));
}
__host__ __device__ inline Real4 rmodr(Real4 a, Real4 b) {
  return make_Real4(rmodr(a.x, b.x), rmodr(a.y, b.y), rmodr(a.z, b.z), rmodr(a.w, b.w));
}

__host__ __device__ inline Real2 fabs(Real2 v) {
  return make_Real2(fabs(v.x), fabs(v.y));
}
__host__ __device__ inline Real3 fabs(Real3 v) {
  return make_Real3(fabs(v.x), fabs(v.y), fabs(v.z));
}
__host__ __device__ inline Real4 fabs(Real4 v) {
  return make_Real4(fabs(v.x), fabs(v.y), fabs(v.z), fabs(v.w));
}

__host__ __device__ inline Real3 reflect(Real3 i, Real3 n) {
  return i - 2.0 * n * dot(n, i);
}

__host__ __device__ inline Real3 cross(Real3 a, Real3 b) {
  return make_Real3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

//#if DOUBLEPRECISION
//__host__ __device__ inline Real smoothstep(Real a, Real b, Real x)
//{
//	Real y = clamp((x - a) / (b - a), 0.0, 1.0);
//	return (y*y*(3.0 - (2.0*y)));
//}
//#endif
//__host__ __device__ inline Real2 smoothstep(Real2 a, Real2 b, Real2 x)
//{
//	Real2 y = clamp((x - a) / (b - a), 0.0, 1.0);
//	return (y*y*(make_Real2(3.0) - (make_Real2(2.0)*y)));
//}
//__host__ __device__ inline Real3 smoothstep(Real3 a, Real3 b, Real3 x)
//{
//	Real3 y = clamp((x - a) / (b - a), 0.0, 1.0);
//	return (y*y*(make_Real3(3.0) - (make_Real3(2.0)*y)));
//}
//__host__ __device__ inline Real4 smoothstep(Real4 a, Real4 b, Real4 x)
//{
//	Real4 y = clamp((x - a) / (b - a), 0.0, 1.0);
//	return (y*y*(make_Real4(3.0) - (make_Real4(2.0)*y)));
//}
////**** some other useful operators
//__host__ __device__ inline bool operator== (const int2 & a , const int2 & b){
//	return (a.x==b.x&&a.y==b.y);
//}

#endif
