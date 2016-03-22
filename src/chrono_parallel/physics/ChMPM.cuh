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

struct MPM_Settings {
    real dt, kernel_radius, inv_radius, bin_edge;
    real inv_bin_edge, max_velocity, mu, lambda;
    real hardening_coefficient, theta_c, theta_s, alpha_flip;
    real youngs_modulus, poissons_ratio;
    int num_mpm_markers;
    int num_mpm_nodes;
    real mass;
    real p1, p2, p3;
    int num_iterations;
    int bins_per_axis_x;
    int bins_per_axis_y;
    int bins_per_axis_z;
};

void MPM_Initialize(MPM_Settings& settings, std::vector<real3>& positions);
void MPM_Solve(MPM_Settings& settings,
               std::vector<real3>& positions,
               std::vector<real3>& velocities,
               std::vector<real>& cohesion);
void MPM_Update_Deformation_Gradient(MPM_Settings& settings, std::vector<real3>& velocities);
}
