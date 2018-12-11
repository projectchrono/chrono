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
// Authors: Dan Negrut, Conlain Kelly
// =============================================================================
#pragma once

#include "chrono_granular/physics/ChGranular.h"

#include "chrono_thirdparty/cub/cub.cuh"

// Print a user-given error message and crash
#define ABORTABORTABORT(...) \
    {                        \
        printf(__VA_ARGS__); \
        __threadfence();     \
        cub::ThreadTrap();   \
    }

/// Get the force multiplier for a contact given the penetration
/// delta_n is penetration normalized by diameter
inline __device__ float get_force_multiplier(float delta_n, GranParamsPtr gran_params) {
    switch (gran_params->force_model) {
        case chrono::granular::GRAN_FORCE_MODEL::HOOKE: {
            return 1.f;
        }
        case chrono::granular::GRAN_FORCE_MODEL::HERTZ: {
            return sqrt(delta_n);
        }
    }
    // if we get here, something is wrong
    ABORTABORTABORT("Invalid contact model\n");
    return 0;  // this should never happen
}
