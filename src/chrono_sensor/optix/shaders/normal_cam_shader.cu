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
// Authors: Bo-Hsun Chen
// =============================================================================
//
// Normal camera shader
//
// =============================================================================

#ifndef NORMAL_CAM_SHADER_CU
#define NORMAL_CAM_SHADER_CU

#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/shaders/shader_utils.cu"

/// @brief Get the PerRayData of the normal camera
/// @return Pointer to the PerRayData_normalCamera struct
__device__ __inline__ PerRayData_normalCamera* GetNormalCameraPRD() {
	unsigned int opt0 = optixGetPayload_0();
	unsigned int opt1 = optixGetPayload_1();
	return reinterpret_cast<PerRayData_normalCamera*>(ints_as_pointer(opt0, opt1));
}

/// @brief Normal camera shader
/// @param prd Pointer to the PerRayData_normalCamera struct
/// @param world_normal Normal vector in world coordinates of the hit point
static __device__ __inline__ void NormalCamShader(PerRayData_normalCamera* prd, const float3& world_normal) {
	prd->normal = world_normal;
}

# endif  // NORMAL_CAM_SHADER_CU