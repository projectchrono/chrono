/* =============================================================================
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
// RT kernels for print exceptions when/if they are caught
//
// ============================================================================= */

#include <math_constants.h>
#include <optixu/optixu_aabb.h>
#include "ray_utils.h"

using namespace optix;

RT_PROGRAM void base() {
    rtPrintExceptionDetails();
    printf("");
}
