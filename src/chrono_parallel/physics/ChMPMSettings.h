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
// MPM settings structure
//
// =============================================================================

#pragma once

namespace chrono {

struct MPM_Settings {
    float dt, kernel_radius, inv_radius, bin_edge;
    float inv_bin_edge, max_velocity, mu, lambda;
    float hardening_coefficient, theta_c, theta_s, alpha_flip;
    float h0, h1, h2, h3;
    float youngs_modulus, poissons_ratio;
    int num_mpm_markers;
    int num_mpm_nodes;
    float mass;
    float yield_stress, p2, p3;
    int num_iterations;
    int bins_per_axis_x;
    int bins_per_axis_y;
    int bins_per_axis_z;
};

}
