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
// Authors: Asher Elmquist, Han Wang, Yan Xiao
// =============================================================================
//
// Segmentation camera shader
//
// =============================================================================

#include "chrono_sensor/optix/shaders/device_utils.h"
#include "chrono_sensor/optix/shaders/shader_utils.cu"

__device__ __inline__ PerRayData_segment* GetSegmentPRD() {
    unsigned int opt0 = optixGetPayload_0();
    unsigned int opt1 = optixGetPayload_1();
    return reinterpret_cast<PerRayData_segment*>(ints_as_pointer(opt0, opt1));
}


static __device__ __inline__ void SegmentCamShader(PerRayData_segment* prd, const MaterialParameters& mat) {
    prd->class_id = mat.class_id;
    prd->instance_id = mat.instance_id;
}