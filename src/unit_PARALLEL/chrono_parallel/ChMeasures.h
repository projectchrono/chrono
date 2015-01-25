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

#ifndef CH_MEASURES_H
#define CH_MEASURES_H

#include "chrono_parallel/ChParallelDefines.h"
#include "parallel/ChOpenMP.h"

namespace chrono {
// collision_measures, like the name implies is the structure that contains all
// measures associated with the collision detection step of chrono parallel
struct collision_measures {
  collision_measures(){
    min_bounding_point = 0;
    max_bounding_point = 0;
    numAABB = 0;
    max_aabb_per_bin = 0;
    global_origin = 0;
    bin_size_vec = 0;
    grid_size = I3(0);

  }
  real3 min_bounding_point; //The minimal global bounding point
  real3 max_bounding_point; //The maximum global bounding point
  uint numAABB;             //The number of AABBs
  uint max_aabb_per_bin;    //The maximum number of AABBs in a bin
  real3 global_origin;      //The global zero point
  real3 bin_size_vec;       //Vector holding bin sizes for each dimension
  int3 grid_size;           //The number of bins in each dimension
};
// solver_measures, like the name implies is the structure that contains all
// measures associated with the parallel solver.
struct solver_measures {
  solver_measures(){
    total_iteration=0;
    residual=0;
    objective_value=0;
  }
  int total_iteration;      // The total number of iterations performed, this variable accumulates
  real residual;            // Current residual for the solver
  real objective_value;     // Current objective value for the solver

  // These three variables are used to store the convergence history of the solver
  custom_vector<real> maxd_hist, maxdeltalambda_hist, iter_hist;

};

struct measures_container {
  collision_measures collision;
  solver_measures solver;
};
}

#endif
