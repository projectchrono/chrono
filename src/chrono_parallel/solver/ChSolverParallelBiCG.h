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
// Implementation of an iterative BiCG solver.
// =============================================================================

#pragma once

#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {

class CH_PARALLEL_API ChSolverParallelBiCG : public ChSolverParallel {
 public:
   ChSolverParallelBiCG() : ChSolverParallel() {}
   ~ChSolverParallelBiCG() {}

  void Solve() {
    if (data_manager->num_constraints == 0) {
      return;
    }
    data_manager->system_timer.start("ChSolverParallel_Solve");
    data_manager->measures.solver.total_iteration += SolveBiCG(
        max_iteration, data_manager->num_constraints, data_manager->host_data.R, data_manager->host_data.gamma);
    data_manager->system_timer.stop("ChSolverParallel_Solve");
  }

  // Solve using the biconjugate gradient method
  uint SolveBiCG(const uint max_iter,            // Maximum number of iterations
                 const uint size,                // Number of unknowns
                 DynamicVector<real>& b,  // Rhs vector
                 DynamicVector<real>& x   // The vector of unknowns
                 );

  DynamicVector<real> z, ztilde, p, ptilde, q, qtilde, r, rtilde;
  real rho_1, rho_2, alpha, beta;
};
}
