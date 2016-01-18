// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Description: This file contains everything that is at some point measured in
// Chrono parallel, things like the broadphase bounding box or the number of
// solver iterations will be stored here.
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "parallel/ChOpenMP.h"

namespace chrono {
// collision_measures, like the name implies is the structure that contains all
// measures associated with the collision detection step of chrono parallel
struct collision_measures {
    collision_measures() {
        min_bounding_point = real3(0);
        max_bounding_point = real3(0);
        global_origin = real3(0);
        bin_size = real3(0);
        number_of_contacts_possible = 0;
        number_of_bins_active = 0;
        number_of_bin_intersections = 0;

        ff_min_bounding_point = real3(0);
        ff_max_bounding_point = real3(0);
        ff_bins_per_axis = int3(0);

        node_min_bounding_point = real3(0);
        node_max_bounding_point = real3(0);
        node_bins_per_axis = int3(0);
    }
    real3 min_bounding_point;          // The minimal global bounding point
    real3 max_bounding_point;          // The maximum global bounding point
    real3 global_origin;               // The global zero point
    real3 bin_size;                    // Vector holding bin sizes for each dimension
    real3 inv_bin_size;                // Vector holding inverse bin sizes for each dimension
    uint number_of_bins_active;        // Number of active bins (containing 1+ AABBs)
    uint number_of_bin_intersections;  // Number of AABB bin intersections
    uint number_of_contacts_possible;  // Number of contacts possible from broadphase

    // Fluid Collision info
    int3 ff_bins_per_axis;
    real3 ff_min_bounding_point;
    real3 ff_max_bounding_point;
    // Tet Collision info
    int3 node_bins_per_axis;
    real3 node_min_bounding_point;
    real3 node_max_bounding_point;
};
// solver_measures, like the name implies is the structure that contains all
// measures associated with the parallel solver.
struct solver_measures {
    solver_measures() {
        total_iteration = 0;
        residual = 0;
        objective_value = 0;

        bilateral_apgd_step_length = 1;
        normal_apgd_step_length = 1;
        sliding_apgd_step_length = 1;
        spinning_apgd_step_length = 1;
        old_objective_value = 0;
    }
    int total_iteration;       // The total number of iterations performed, this variable accumulates
    real residual;             // Current residual for the solver
    real objective_value;      // Current objective value for the solver
    real old_objective_value;  // Objective value from the previous iter

    real bilateral_apgd_step_length;
    real normal_apgd_step_length;
    real sliding_apgd_step_length;
    real spinning_apgd_step_length;

    // These three variables are used to store the convergence history of the solver
    std::vector<real> maxd_hist, maxdeltalambda_hist, time;
    std::vector<real> apgd_beta;
    std::vector<real> apgd_step;
    std::vector<real> apgd_step_time;
    std::vector<real> violation;
};

struct measures_container {
    collision_measures collision;
    solver_measures solver;
};
}
