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

#define CAMERA_RAY_TYPE 0
#define SHADOW_RAY_TYPE 1
#define LIDAR_RAY_TYPE 2

struct PerRayData_camera {
    float3 color;
    float importance;
    int depth;
};

struct PerRayData_lidar {
    float range;
    float intensity;
    int depth;
};

static __device__ __inline__ PerRayData_camera make_camera_data(const float3& r_color,
                                                                const float& r_importance,
                                                                const int& r_depth) {
    PerRayData_camera ray_data;
    ray_data.color = r_color;
    ray_data.importance = r_importance;
    ray_data.depth = r_depth;
    return ray_data;
}

static __device__ __inline__ PerRayData_lidar make_lidar_data(const float& r_range,
                                                              const float& r_intensity,
                                                              const int& r_depth) {
    PerRayData_lidar ray_data;
    ray_data.range = r_range;
    ray_data.intensity = r_intensity;
    ray_data.depth = r_depth;
    return ray_data;
}

struct PerRayData_shadow {
    float3 attenuation;
};

static __device__ __inline__ PerRayData_shadow make_shadow_data(const float3& r_attenuation) {
    //
    PerRayData_shadow ray_data;
    ray_data.attenuation = r_attenuation;
    return ray_data;
}

static __device__ __inline__ uchar4 make_color(const float3& c) {
    return make_uchar4(static_cast<unsigned char>(__saturatef(c.x) * 255.9999f),
                       static_cast<unsigned char>(__saturatef(c.y) * 255.9999f),
                       static_cast<unsigned char>(__saturatef(c.z) * 255.9999f), 255);
}

static __device__ __inline__ float4 make_float4(const float3& point, const uchar4& color) {
    float csf;
    memcpy(&csf, &color, sizeof(csf));

    return make_float4(point.x, point.y, point.z, csf);
}

static __device__ __inline__ float sensor_rand(unsigned int& seed) {
    seed = (1103515245u * seed + 12345u) % 2147483648u;
    return (float)(seed) / (float)(2147483648u);
}
