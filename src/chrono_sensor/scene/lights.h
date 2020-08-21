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
// Light definitions
//
// =============================================================================

#ifndef LIGHTS_H
#define LIGHTS_H

#ifdef _WIN32
 #define NOMINMAX
#endif

#include <optix.h>
#include <optixu/optixu_math_namespace.h>

/// @addtogroup sensor_scene
/// @{

struct PointLight {
    optix::float3 pos;    ///< position of the light in global coordinate frame
    optix::float3 color;  ///< color of the light, encodes intensity as well
    float max_range;      ///< range of the point light, where maximum range equates to 1% remaining light intensity
};

/// @} sensor_scene

#endif
