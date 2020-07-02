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

#include <optix.h>
#include <optixu/optixu_math_namespace.h>

struct PointLight {
    optix::float3 pos;
    optix::float3 color;
    float max_range;
    int casts_shadow;
};

#endif
