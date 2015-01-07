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
// Description: This file contains all of the setting structures that are used
// within chrono parallel. THe constructor for each struct should specify
// default values for every setting parameter
// =============================================================================

#ifndef CH_SETTINGS_H
#define CH_SETTINGS_H

#include "chrono_parallel/ChParallelDefines.h"
#include "parallel/ChOpenMP.h"

namespace chrono {
struct collision_settings {
  collision_settings() {
    max_body_per_bin = 50;
    min_body_per_bin = 25;
    use_aabb_active = 0;
    collision_envelope = 0;
    bins_per_axis = I3(20, 20, 20);
    narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
    // edge_radius = 0.1;
  }

  // Collision variables
  real3 min_bounding_point, max_bounding_point;
  real collision_envelope;
  bool use_aabb_active;
  int min_body_per_bin;
  int max_body_per_bin;
  real3 aabb_min, aabb_max;
  int3 bins_per_axis;
  NARROWPHASETYPE narrowphase_algorithm;
  // real edge_radius;
};

struct solver_settings {

  solver_settings() {
    tolerance = 1e-4;
    tolerance_objective = 1e-6;
    collision_in_solver = false;
    update_rhs = false;
    verbose = false;
    test_objective = false;

    alpha = .2;
    contact_recovery_speed = .6;
    bilateral_clamp_speed = .6;
    clamp_bilaterals = true;
    perform_stabilization = false;
    collision_in_solver = false;

    max_iteration = 100;
    max_iteration_normal = 0;
    max_iteration_sliding = 100;
    max_iteration_spinning = 0;
    max_iteration_bilateral = 100;

    solver_type = APGD;
    solver_mode = SLIDING;
    step_size = .01;
  }

  SOLVERTYPE solver_type;
  SOLVERMODE solver_mode;
  real alpha;

  real contact_recovery_speed;
  real bilateral_clamp_speed;
  bool clamp_bilaterals;
  bool perform_stabilization;
  bool collision_in_solver;
  bool update_rhs;
  bool verbose;
  bool test_objective;

  uint max_iteration;
  uint max_iteration_normal;
  uint max_iteration_sliding;
  uint max_iteration_spinning;
  uint max_iteration_bilateral;

  // Solver variables
  real step_size;
  real tolerance;
  real tolerance_objective;
};

struct settings_container {

  settings_container() {
    min_threads = 1;
    max_threads = CHOMPfunctions::GetNumProcs();
    // Only perform thread tuning if max threads is greater than min_threads;
    // I don't really check to see if max_threads is > than min_threads
    // not sure if that is a huge issue
    perform_thread_tuning = ((min_threads == max_threads) ? false : true);
    perform_bin_tuning = true;
    system_type = SYSTEM_DVI;
    bin_perturb_size = 2;    // increase or decrease the number of bins by 2
    bin_tuning_frequency = 20;    // bins will be tuned every 20 frames
  }

  // CD Settings
  collision_settings collision;
  // Solver Settings
  solver_settings solver;
  // System Settings
  bool perform_thread_tuning;
  bool perform_bin_tuning;
  int bin_perturb_size;
  int bin_tuning_frequency;
  int min_threads;
  int max_threads;
  SYSTEMTYPE system_type;
};
}

#endif
