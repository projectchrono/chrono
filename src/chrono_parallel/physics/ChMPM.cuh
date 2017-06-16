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

#include <vector>
#include "chrono_parallel/physics/ChMPMSettings.h"

namespace chrono {

void MPM_Initialize(MPM_Settings& settings, std::vector<float>& positions);
void MPM_Solve(MPM_Settings& settings, std::vector<float>& positions, std::vector<float>& velocities);

void MPM_UpdateDeformationGradient(MPM_Settings& settings,
                                   std::vector<float>& positions,
                                   std::vector<float>& velocities,
                                   std::vector<float>& jejp);
}
