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

#include "../ChApiGranular.h"
#include "chrono_granular/ChGranularDefines.h"
#include "cuda_runtime.h"

namespace chrono {
namespace granular {

struct sphere_BC_params_t {
    int3 sphere_center;
    unsigned int radius;
    int normal_sign;
};
/// Z-oriented cone facing upward
struct cone_BC_params_t {
    int3 cone_tip;
    float slope;
    int hmax;
    int hmin;
    int normal_sign;
};
/// Axis-aligned box
struct AABox_BC_params_t {
    int3 max_corner;
    int3 min_corner;
    int normal_sign;
};

union BC_params_t {
    AABox_BC_params_t AABox_params;
    sphere_BC_params_t sphere_params;
    cone_BC_params_t cone_params;
};

enum BC_type { SPHERE, AA_BOX, CONE };

typedef bool (*BCfunc_t)(const int, const int, const int, float&, float&, float&, const BC_params_t&, ParamsPtr);

}  // namespace granular
}  // namespace chrono
