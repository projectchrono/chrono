/* =============================================================================
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
// RT kernels for coloring upon ray not intersecting anything
//
// ============================================================================= */

#include <math_constants.h>
#include <optixu/optixu_aabb.h>
#include "ray_utils.h"

using namespace optix;

rtDeclareVariable(PerRayData_camera, prd_camera, rtPayload, );
rtDeclareVariable(PerRayData_lidar, prd_lidar, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float3, default_color, , );
rtDeclareVariable(float, default_depth, , );

// environment map
rtTextureSampler<float4, 2> environment_map;
rtDeclareVariable(int, has_environment_map, , );

RT_PROGRAM void camera_miss() {
    if (has_environment_map) {
        float theta = atan2f(ray.direction.x, ray.direction.y);
        float phi = asinf(ray.direction.z);
        float tex_x = theta / (2 * M_PIf);
        float tex_y = phi / (M_PIf) + 0.5;

        prd_camera.color = make_float3(tex2D(environment_map, tex_x, tex_y));

    } else {
        prd_camera.color = default_color;
    }
}

RT_PROGRAM void lidar_miss() {
    prd_lidar.range = default_depth;
    prd_lidar.intensity = 0.f;
}
