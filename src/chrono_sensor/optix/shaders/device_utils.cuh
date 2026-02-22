// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef DEVICE_UTILS_CUH
#define DEVICE_UTILS_CUH

#include "chrono_sensor/optix/ChOptixDefinitions.h"

#include <math_constants.h>
#include <optix.h>

typedef uchar3				bool3;
typedef unsigned char		uchar;
typedef unsigned short		ushort;
typedef unsigned int		uint;
typedef unsigned long		ulong;
typedef unsigned long long	uint64;

#define INV_PI 1/CUDART_PI

extern "C" {
    __constant__ ContextParameters params;
}

__device__ __inline__ half4 make_half4(const float4& a) {
    return {__float2half(a.x), __float2half(a.y), __float2half(a.z), __float2half(a.w)};
}

__device__ __inline__ half4 make_half4(const float& a, const float& b, const float& c, const float& d) {
    return {__float2half(a), __float2half(b), __float2half(c), __float2half(d)};
}

static __device__ __inline__ uchar4 make_color(const float3& c) {
    return make_uchar4(static_cast<unsigned char>(__saturatef(c.x) * 255.9999f),
                       static_cast<unsigned char>(__saturatef(c.y) * 255.9999f),
                       static_cast<unsigned char>(__saturatef(c.z) * 255.9999f), 255);
}

static __device__ __inline__ void* ints_as_pointer(unsigned int& a, unsigned int& b) {
    const unsigned long long ptr = static_cast<unsigned long long>(a) << 32 | b;
    return reinterpret_cast<void*>(ptr);
}

static __device__ __inline__ void pointer_as_ints(void* ptr, unsigned int& a, unsigned int& b) {
    const unsigned long long uptr = reinterpret_cast<unsigned long long>(ptr);
    a = uptr >> 32;
    b = uptr & 0x00000000ffffffff;
}

static __device__ __inline__ float sensor_rand(unsigned int seed) {
    unsigned int next = (1103515245u * seed + 12345u) % 2147483648u;
    return (float)(next) / (float)(2147483648u);
}

static __device__ __inline__ float sensor_next_rand(float& rand) {
    unsigned int next_seed = (unsigned int)(rand * 2147483648u);
    float next_rand = sensor_rand(next_seed);
    rand = next_rand;
    return rand;
}

/// =======================
/// float3-float3 operators
/// =======================
__device__ __inline__ float3 operator+(const float3& a, const float3& b) {
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__device__ __inline__ void operator+=(float3& a, const float3& b) {
    a = a + b;
}

__device__ __inline__ float3 operator-(const float3& a, const float3& b) {
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ __inline__ float3 operator-(const float3& a) {
    return make_float3(-a.x, -a.y, -a.z);
}

__device__ __inline__ float3 operator/(const float3& a, const float3& b) {
    return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
}

__device__ __inline__ float3 operator*(const float3& a, const float3& b) {
    return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}

__device__ __inline__ bool operator==(const float3& a, const float3& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

/// =======================
/// float4-float4 operators
/// =======================
__device__ __inline__ float4 operator+(const float4& a, const float4& b) {
    return make_float4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

__device__ __inline__ void operator+=(float4& a, const float4& b) {
    a = a + b;
}

__device__ __inline__ float4 operator-(const float4& a, const float4& b) {
    return make_float4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

__device__ __inline__ float4 operator-(const float4& a) {
    return make_float4(-a.x, -a.y, -a.z, -a.w);
}

__device__ __inline__ float4 operator/(const float4& a, const float4& b) {
    return make_float4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}

__device__ __inline__ float4 operator*(const float4& a, const float4& b) {
    return make_float4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}

/// ================
/// vector functions
/// ================
__device__ __inline__ float Dot(const float4& v1, const float4& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
}

__device__ __inline__ float Dot(const float3& v1, const float3& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

__device__ __inline__ float Dot(const float2& v1, const float2& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

__device__ __inline__ float3 Cross(const float3& v1, const float3& v2) {
    return make_float3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

__device__ __inline__ float Length(const float4& v) {
    return sqrt(Dot(v, v));
}

__device__ __inline__ float Length(const float3& v) {
    return sqrt(Dot(v, v));
}

__device__ __inline__ float Length(const float2& v) {
    return sqrt(Dot(v, v));
}

__device__ __inline__ float fmaxf(const float3& a) {
    return fmaxf(a.x, fmaxf(a.y, a.z));
}

__device__ __inline__ float fminf(const float3& a) {
    return fminf(a.x, fminf(a.y, a.z));
}

__device__ __inline__ float3 make_float3(const float& a) {
    return make_float3(a, a, a);
}

__device__ __inline__ float3 make_float3(const float4& a) {
    return make_float3(a.x, a.y, a.z);
}

__device__ __inline__ float3 Pow(const float3& v, const float& a) {
    return make_float3(pow(v.x, a), pow(v.y, a), pow(v.z, a));
}
/// =======================
/// float3-float operators
/// =======================
__device__ __inline__ float3 operator*(const float& a, const float3& v) {
    return make_float3(a * v.x, a * v.y, a * v.z);
}

__device__ __inline__ float3 operator*(const float3& v, const float& a) {
    return make_float3(a * v.x, a * v.y, a * v.z);
}

__device__ __inline__ float3 operator/(const float3& v, const float& a) {
    const float inv = 1 / a;
    return make_float3(inv * v.x, inv * v.y, inv * v.z);
}

__device__ __inline__ float3 normalize(const float3& a) {
    return a / Length(a);
}

/// =======================
/// float4-float operators
/// =======================
__device__ __inline__ float4 operator*(const float& a, const float4& v) {
    return make_float4(a * v.x, a * v.y, a * v.z, a * v.w);
}

__device__ __inline__ float4 operator*(const float4& v, const float& a) {
    return make_float4(a * v.x, a * v.y, a * v.z, a * v.w);
}

__device__ __inline__ float4 operator/(const float4& v, const float& a) {
    const float inv = 1 / a;
    return make_float4(inv * v.x, inv * v.y, inv * v.z, inv * v.w);
}

__device__ __inline__ float4 normalize(const float4& a) {
    return a / Length(a);
}

/// ======================
/// float3-float functions
/// ======================
__device__ __inline__ float3 fmaxf(const float3& a, const float3& b) {
    return make_float3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}

__device__ __inline__ float3 fminf(const float3& a, const float3& b) {
    return make_float3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}

/// =======================
/// float2-float2 operators
/// =======================
__device__ __inline__ float2 operator+(const float2& a, const float2& b) {
    return make_float2(a.x + b.x, a.y + b.y);
}

__device__ __inline__ float2 operator-(const float2& a, const float2& b) {
    return make_float2(a.x - b.x, a.y - b.y);
}

__device__ __inline__ float2 operator-(const float2& a) {
    return make_float2(-a.x, -a.y);
}

__device__ __inline__ float2 operator/(const float2& a, const float2& b) {
    return make_float2(a.x / b.x, a.y / b.y);
}

/// =======================
/// float2-float operators
/// =======================
__device__ __inline__ float2 operator*(const float& a, const float2& v) {
    return make_float2(a * v.x, a * v.y);
}

__device__ __inline__ float2 operator*(const float2& v, const float& a) {
    return make_float2(a * v.x, a * v.y);
}

__device__ __inline__ float2 operator/(const float2& v, const float& a) {
    const float inv = 1 / a;
    return make_float2(inv * v.x, inv * v.y);
}

/// ================
/// float2 functions
/// ================
__device__ __inline__ float2 make_float2(const float& a) {
    return make_float2(a, a);
}

__device__ __inline__ float2 make_float2(const int2& a) {
    return make_float2((float)a.x, (float)a.y);
}

__device__ __inline__ float2 normalize(const float2& a) {
    return a / Length(a);
}

/// ================
/// clamp functions
/// ================
__device__ __inline__ float clamp(const float& a, const float& low, const float& high) {
    return max(low, min(a, high));
}

__device__ __inline__ float2 clamp(const float2& a, const float2& low, const float2& high) {
    return make_float2(clamp(a.x, low.x, high.x), clamp(a.y, low.y, high.y));
}

__device__ __inline__ float3 clamp(const float3& a, const float3& low, const float3& high) {
    return make_float3(clamp(a.x, low.x, high.x), clamp(a.y, low.y, high.y), clamp(a.z, low.z, high.z));
}

/// ==================
/// graphics functions
/// ==================
__device__ __inline__ float fresnel_schlick(const float& cos,
                                            const float& exp = 5.f,
                                            const float& min = 0.f,
                                            const float& max = 1.f) {
    return clamp(min + (max - min) * powf(fmaxf(0.f, 1.f - cos), exp), min, max);
}

__device__ __inline__ float3 fresnel_schlick(const float& cos, const float& exp, const float3& min, const float3& max) {
    return make_float3(fresnel_schlick(cos, exp, min.x, max.x), fresnel_schlick(cos, exp, min.y, max.y),
                       fresnel_schlick(cos, exp, min.z, max.z));
}

__device__ __inline__ float luminance(const float3& color) {
    static const float3 l = {0.30f, 0.59f, 0.11f};
    return Dot(color, l);
}

// Assume v as in-coming into the surface, and returned vector is going out from the surface
__device__ __inline__ float3 reflect(const float3& v, const float3& n) {
    return 2 * Dot(n, -v) * n + v;
}

__device__ __inline__ float3 refract(const float3& v, const float3& n, const float& n1, const float& n2) {
    float n_ratio = n1 / n2;
    float cosi = -Dot(n, v);
    return n_ratio * v + (n_ratio * cosi - sqrtf(max(0.f, 1 - n_ratio * n_ratio * cosi * cosi))) * n;
}

/// @brief Convert a quaternion to an orthonormal basis.
/// @param q The input quaternion, in (x, y, z, w) format where w is the scalar part.
/// @param x The returned output x-axis of the basis.
/// @param y The returned output y-axis of the basis.
/// @param z The returned output z-axis of the basis.
__device__ __inline__ void basis_from_quaternion(const float4& q, float3& x, float3& y, float3& z) {
    const float e0e0 = q.x * q.x;
    const float e1e1 = q.y * q.y;
    const float e2e2 = q.z * q.z;
    const float e3e3 = q.w * q.w;
    const float e0e1 = q.x * q.y;
    const float e0e2 = q.x * q.z;
    const float e0e3 = q.x * q.w;
    const float e1e2 = q.y * q.z;
    const float e1e3 = q.y * q.w;
    const float e2e3 = q.z * q.w;
    x = make_float3((e0e0 + e1e1) * 2.f - 1.f, (e1e2 + e0e3) * 2.f, (e1e3 - e0e2) * 2.f);
    y = make_float3((e1e2 - e0e3) * 2.f, (e0e0 + e2e2) * 2.f - 1.f, (e2e3 + e0e1) * 2.f);
    z = make_float3((e1e3 + e0e2) * 2.f, (e2e3 - e0e1) * 2.f, (e0e0 + e3e3) * 2.f - 1.f);
}

__device__ __inline__
float4 quaternion_from_basis(const float3& x, const float3& y, const float3& z) {
    // Rotation matrix elements (column-major)
    const float m00 = x.x, m01 = y.x, m02 = z.x;
    const float m10 = x.y, m11 = y.y, m12 = z.y;
    const float m20 = x.z, m21 = y.z, m22 = z.z;

    float4 q;

    const float trace = m00 + m11 + m22;

    if (trace > 0.0f) {
        const float s = sqrtf(trace + 1.0f) * 2.0f;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
        q.w = 0.25f * s;
    }
    else if (m00 > m11 && m00 > m22) {
        const float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        q.x = 0.25f * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
        q.w = (m21 - m12) / s;
    }
    else if (m11 > m22) {
        const float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        q.x = (m01 + m10) / s;
        q.y = 0.25f * s;
        q.z = (m12 + m21) / s;
        q.w = (m02 - m20) / s;
    }
    else {
        const float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25f * s;
        q.w = (m10 - m01) / s;
    }

    return q;
}

__device__ __inline__ float lerp(const float& a, const float& b, const float& t) {
    return a + t * (b - a);
}

__device__ __inline__ float2 lerp(const float2& a, const float2& b, const float& t) {
    return a + t * (b - a);
}

__device__ __inline__ float3 lerp(const float3& a, const float3& b, const float& t) {
    return a + t * (b - a);
}

__device__ __inline__ float4 lerp(const float4& a, const float4& b, const float& t) {
    return a + t * (b - a);
}

__device__ __inline__ float2 nlerp(const float2& a, const float2& b, const float& t) {
    return normalize(lerp(a, b, t));
}

__device__ __inline__ float3 nlerp(const float3& a, const float3& b, const float& t) {
    return normalize(lerp(a, b, t));
}

__device__ __inline__ float4 nlerp(const float4& a, const float4& b, const float& t) {
    return normalize(lerp(a, b, t));
}

__device__ __inline__ float radial_function(const float& rd2, const LensParams& params){
    // Drap, P., & Lefevre, J. (2016). 
    // An Exact Formula for Calculating Inverse Radial Lens Distortions. 
    // Sensors (Basel, Switzerland), 16(6), 807. https://doi.org/10.3390/s16060807
    double rd4 = rd2 * rd2;
    double rd6 = rd4 * rd2;
    double rd8 = rd4 * rd4;
    double rd10 = rd6 * rd4;
    double rd12 = rd6 * rd6;
    double rd14 = rd8 * rd6;
    double rd16 = rd8 * rd8;
    double rd18 = rd10 * rd8;

    float ru = (float)(1.0 + params.a0 * rd2 + 
        params.a1 * rd4 +
        params.a2 * rd6 + 
        params.a3 * rd8 +
        params.a4 * rd10 +
        params.a5 * rd12 +
        params.a6 * rd14 +
        params.a7 * rd16 +
        params.a8 * rd18);
    return ru;
}

__device__ __inline__ float gaussian(int x, int y, float sigma) {
    return expf(-(x * x + y * y) / (2 * sigma * sigma)) / (2 * CUDART_PI_F * sigma * sigma);
}


#ifdef USE_SENSOR_NVDB
    __device__ __inline__ float3 make_float3(const nanovdb::Vec3f& a) {
        return make_float3(a[0], a[1], a[2]);
    }

    __device__ __inline__ nanovdb::Vec3f make_nanovec3f(const float3& a) {
        return nanovdb::Vec3f(a.x,a.y,a.z);
    }
#endif

#endif
