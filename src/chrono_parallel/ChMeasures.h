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
// Description: This file contains everything that is at some point measured in
// Chrono parallel, things like the broadphase bounding box or the number of
// solver iterations will be stored here.
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono/parallel/ChOpenMP.h"
#include "chrono_parallel/math/other_types.h"  // for uint, vec2, vec3
#include "chrono_parallel/math/real.h"         // for real
#include "chrono_parallel/math/real2.h"        // for real2
#include "chrono_parallel/math/real3.h"        // for real3
#include "chrono_parallel/math/real4.h"        // for quaternion, real4

namespace chrono {

/// @addtogroup parallel_module
/// @{

/// Collision_measures.
/// This structure contains all measures associated with the collision detection step of chrono parallel.
class collision_measures {
  public:
    collision_measures() {
        min_bounding_point = real3(0);
        max_bounding_point = real3(0);
        global_origin = real3(0);
        bin_size = real3(0);
        number_of_contacts_possible = 0;
        number_of_bins_active = 0;
        number_of_bin_intersections = 0;

        rigid_min_bounding_point = real3(0);
        rigid_max_bounding_point = real3(0);

        ff_min_bounding_point = real3(0);
        ff_max_bounding_point = real3(0);
        ff_bins_per_axis = vec3(0);

        tet_min_bounding_point = real3(0);
        tet_max_bounding_point = real3(0);
        tet_bins_per_axis = vec3(0);

        mpm_min_bounding_point = real3(0);
        mpm_max_bounding_point = real3(0);
        mpm_bins_per_axis = vec3(0);
    }
    real3 min_bounding_point;          ///< The minimal global bounding point
    real3 max_bounding_point;          ///< The maximum global bounding point
    real3 global_origin;               ///< The global zero point
    real3 bin_size;                    ///< Vector holding bin sizes for each dimension
    real3 inv_bin_size;                ///< Vector holding inverse bin sizes for each dimension
    uint number_of_bins_active;        ///< Number of active bins (containing 1+ AABBs)
    uint number_of_bin_intersections;  ///< Number of AABB bin intersections
    uint number_of_contacts_possible;  ///< Number of contacts possible from broadphase

    real3 rigid_min_bounding_point;
    real3 rigid_max_bounding_point;

    // Fluid Collision info
    vec3 ff_bins_per_axis;
    real3 ff_min_bounding_point;
    real3 ff_max_bounding_point;
    // Tet Collision info
    vec3 tet_bins_per_axis;
    real3 tet_min_bounding_point;
    real3 tet_max_bounding_point;

    // MPM Collision info

    real3 mpm_min_bounding_point;
    real3 mpm_max_bounding_point;
    vec3 mpm_bins_per_axis;
};

/// Solver measures.
/// This structure contains all measures associated with the parallel solver.
class solver_measures {
  public:
    solver_measures() {
        total_iteration = 0;
        residual = 0;
        objective_value = 0;

        bilateral_apgd_step_length = 1;
        normal_apgd_step_length = 1;
        sliding_apgd_step_length = 1;
        spinning_apgd_step_length = 1;
        old_objective_value = 0;
        lambda_max = 0;
    }
    int total_iteration;       ///< The total number of iterations performed, this variable accumulates
    real residual;             ///< Current residual for the solver
    real objective_value;      ///< Current objective value for the solver
    real old_objective_value;  ///< Objective value from the previous iter

    real bilateral_apgd_step_length;
    real normal_apgd_step_length;
    real sliding_apgd_step_length;
    real spinning_apgd_step_length;
    real lambda_max;  ///< Largest eigenvalue

    // These three variables are used to store the convergence history of the solver
    std::vector<real> maxd_hist, maxdeltalambda_hist, time;

    std::vector<real> apgd_beta;
    std::vector<real> apgd_step;
    std::vector<real> apgd_step_time;
    std::vector<real> violation;
};

/// Aggregate of collision and solver measures.
class measures_container {
  public:
    collision_measures collision;
    solver_measures solver;
};

/// @} parallel_module

}  // end namespace chrono
