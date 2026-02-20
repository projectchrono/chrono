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
// Authors: Nevindu M. Batagoda
// =============================================================================
//
// Depth camera shader
//
// =============================================================================

#ifndef DEPTH_CAM_SHADER_CU
#define DEPTH_CAM_SHADER_CU

#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/shaders/shader_utils.cu"
#include "chrono_sensor/optix/shaders/depth_cam_raygen.cu"

__device__ __inline__ PerRayData_depthCamera* GetDepthCameraPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_depthCamera*>(ints_as_pointer(opt0, opt1));
}

static __device__ __inline__ void DepthCamShader(PerRayData_depthCamera* prd, const float& ray_dist) {
    prd->depth = fminf(prd->max_depth, ray_dist);
}

#endif  // DEPTH_CAM_SHADER_CU