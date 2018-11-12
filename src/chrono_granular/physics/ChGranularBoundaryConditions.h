// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut, Nic Olsen
// =============================================================================
/*! \file */

#pragma once

#include "chrono_granular/ChApiGranular.h"
#include "chrono_granular/ChGranularDefines.h"
#include "cuda_runtime.h"

namespace chrono {
namespace granular {

template <typename T, typename T3>
struct sphere_BC_params_t {
    T3 sphere_center;
    unsigned int radius;
    T normal_sign;
};

/// Z-oriented cone facing upward
template <typename T, typename T3>
struct cone_BC_params_t {
    T3 cone_tip;
    float slope;
    T hmax;
    T hmin;
    T normal_sign;
};

/// Axis-aligned box
template <typename T, typename T3>
struct AABox_BC_params_t {
    T3 max_corner;
    T3 min_corner;
    T normal_sign;
};

/// Infinite Plane
template <typename T3>
struct Plane_BC_params_t {
    float3 normal;
    T3 position;
};

template <typename T, typename T3>
struct BC_params_t {
    bool active;
    union {
        AABox_BC_params_t<T, T3> AABox_params;
        sphere_BC_params_t<T, T3> sphere_params;
        cone_BC_params_t<T, T3> cone_params;
        Plane_BC_params_t<T3> plane_params;  // plane only needs one arg
    };
};

enum BC_type { SPHERE, AA_BOX, CONE, PLANE };
}  // namespace granular
}  // namespace chrono
