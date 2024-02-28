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

#ifndef CHSENSORDEVICEUTILS_H
#define CHSENSORDEVICEUTILS_H

#include "chrono_sensor/optix/ChOptixDefinitions.h"

#include <math_constants.h>
#include <optix.h>

typedef uchar3				bool3;
typedef unsigned char		uchar;
typedef unsigned short		ushort;
typedef unsigned int		uint;
typedef unsigned long		ulong;
typedef unsigned long long	uint64;

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

__device__ __inline__ PerRayData_camera* getCameraPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_camera*>(ints_as_pointer(opt0, opt1));
}

__device__ __inline__ PerRayData_semantic* getSemanticPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_semantic*>(ints_as_pointer(opt0, opt1));
}

__device__ __inline__ PerRayData_depthCamera* getDepthCameraPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_depthCamera*>(ints_as_pointer(opt0, opt1));
}


__device__ __inline__ PerRayData_lidar* getLidarPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_lidar*>(ints_as_pointer(opt0, opt1));
}

__device__ __inline__ PerRayData_radar* getRadarPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_radar*>(ints_as_pointer(opt0, opt1));
}

__device__ __inline__ PerRayData_shadow* getShadowPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_shadow*>(ints_as_pointer(opt0, opt1));
}

__device__ __inline__ PerRayData_camera default_camera_prd() {
    PerRayData_camera prd = {};
    prd.color = make_float3(0.f, 0.f, 0.f);
    prd.contrib_to_pixel = make_float3(1.f, 1.f, 1.f);
    prd.rng = curandState_t();
    prd.depth = 2;
    prd.use_gi = false;
    prd.albedo = make_float3(0.f, 0.f, 0.f);
    prd.normal = make_float3(0.f, 0.f, 0.f);
    prd.use_fog = true;
    return prd;
};


__device__ __inline__ PerRayData_depthCamera default_depthCamera_prd() {
    PerRayData_depthCamera prd = {};
    prd.depth = 0.f;
    return prd;
};

__device__ __inline__ PerRayData_semantic default_semantic_prd() {
    PerRayData_semantic prd = {};
    prd.class_id = 0;
    prd.instance_id = 0;
    return prd;
};

__device__ __inline__ PerRayData_shadow default_shadow_prd() {
    PerRayData_shadow prd = {make_float3(1.f, 1.f, 1.f),  // default opacity amount
                             3,                           // default depth
                             0.f};                        // default distance remaining to light
    return prd;
};

__device__ __inline__ PerRayData_lidar default_lidar_prd() {
    PerRayData_lidar prd = {
        0.f,  // default range
        0.f   // default intensity
    };
    return prd;
};
__device__ __inline__ PerRayData_radar default_radar_prd() {
    PerRayData_radar prd = {
        0.f,  // default range
        0.f   // default rcs
    };
    return prd;
};
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
    const float3 l = {0.30f, 0.59f, 0.11f};
    return Dot(color, l);
}

// assumes v in coming into the surface
__device__ __inline__ float3 reflect(const float3& v, const float3& n) {
    return 2 * Dot(n, -v) * n + v;
}

__device__ __inline__ float3 refract(const float3& v, const float3& n, const float& n1, const float& n2) {
    float n_ratio = n1 / n2;
    float cosi = -Dot(n, v);
    return n_ratio * v + (n_ratio * cosi - sqrtf(max(0.f, 1 - n_ratio * n_ratio * cosi * cosi))) * n;
}

__device__ __inline__ void basis_from_quaternion(const float4& q, float3& f, float3& g, float3& h) {
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
    f = make_float3((e0e0 + e1e1) * 2.f - 1.f, (e1e2 + e0e3) * 2.f, (e1e3 - e0e2) * 2.f);
    g = make_float3((e1e2 - e0e3) * 2.f, (e0e0 + e2e2) * 2.f - 1.f, (e2e3 + e0e1) * 2.f);
    h = make_float3((e1e3 + e0e2) * 2.f, (e2e3 - e0e1) * 2.f, (e0e0 + e3e3) * 2.f - 1.f);
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

__device__ __inline__ float3 sample_hemisphere_dir(const float& z1, const float& z2, const float3& normal) {
    const float radius = sqrtf(z1);
    const float theta = 2.f * CUDART_PI_F * z2;
    float x = radius * cosf(theta);
    float y = radius * sinf(theta);
    float z = sqrtf(fmaxf(0.f, 1.f - x * x - y * y));
    float3 binormal = make_float3(0);

    // Prevent normal = (0, 0, 1)
    if (fabs(normal.x) > fabs(normal.z)) {
        binormal.x = -normal.y;
        binormal.y = normal.x;
        binormal.z = 0;
    } else {
        binormal.x = 0;
        binormal.y = -normal.z;
        binormal.z = normal.y;
    }

    // float3 binormal = make_float3(-normal.y, normal.x, 0);
    float3 tangent = Cross(normal, binormal);
    return x * tangent + y * binormal + z * normal;
}

#endif
