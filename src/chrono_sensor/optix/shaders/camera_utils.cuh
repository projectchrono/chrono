// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bo-Hsun Chen
// =============================================================================

#ifndef CAMERA_UTILS_CUH
#define CAMERA_UTILS_CUH

#include "chrono_sensor/optix/ChOptixDefinitions.h"

/// @brief Initialize a PerRayData_camera struct with default values for camera ray generation
/// @return A PerRayData_camera struct with default values
__device__ __inline__ PerRayData_camera DefaultCameraPRD() {
    PerRayData_camera prd = {};
    prd.color = make_float3(0.f, 0.f, 0.f);
    prd.contrib_to_pixel = make_float3(1.f, 1.f, 1.f);
    prd.rng = curandState_t();
    prd.depth = 2;
    prd.use_gi = false;
    prd.albedo = make_float3(0.f, 0.f, 0.f);
    prd.normal = make_float3(0.f, 0.f, 0.f);
    prd.use_fog = false;
    prd.transparency = 1.f;
    prd.integrator = Integrator::LEGACY;
    return prd;
};

#endif  // CAMERA_UTILS_CUH
