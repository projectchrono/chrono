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
// RT kernels for box geometries
//
// =============================================================================

#ifdef _WIN32
 #define NOMINMAX
#endif

#include <optix.h>
#include <optixu/optixu_math_namespace.h>

#include "chrono_sensor/rtkernels/ray_utils.h"
#include "chrono_sensor/scene/lights.h"

using namespace optix;

rtDeclareVariable(PerRayData_shadow, prd_shadow, rtPayload, );

RT_PROGRAM void hit_shadow() {
    // if the shadow ray hits anything before reaching the light, light clearly cannot hit this point
    prd_shadow.attenuation = make_float3(0.0f);
    rtTerminateRay();
}
