// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Class for the Pure MPM solve, takes positions and velocities
// as input, outputs updated positions and velocities
// =============================================================================

#include "chrono_parallel/math/matrix.h"
#include "chrono_parallel/math/other_types.h"
#include <vector>

namespace chrono {

void MPM_Initialize(const real marker_mass, const real radius, std::vector<real3>& positions);
void MPM_Solve(const real kernel_radius, std::vector<real3>& positions, std::vector<real3>& velocities);
}
