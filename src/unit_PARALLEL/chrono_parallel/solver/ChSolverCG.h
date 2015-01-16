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
// Implementation of an iterative Conjugate Gradient solver.
// =============================================================================

#ifndef CHSOLVERCG_H
#define CHSOLVERCG_H

#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {

class CH_PARALLEL_API ChSolverCG : public ChSolverParallel {
public:
  ChSolverCG() : ChSolverParallel() {}
  ~ChSolverCG() {}

  void Solve() {
    if (data_container->num_constraints == 0) {
      return;
    }
    data_container->system_timer.start("ChSolverParallel_Solve");
    data_container->measures.solver.total_iteration +=
      SolveCG(max_iteration,
              data_container->num_constraints,
              data_container->host_data.R,
              data_container->host_data.gamma);
    data_container->system_timer.stop("ChSolverParallel_Solve");
  }

  // Solve using the conjugate gradient method
  uint SolveCG(const uint max_iter,              // Maximum number of iterations
               const uint size,                  // Number of unknowns
               blaze::DynamicVector<real>& b,    // Rhs vector
               blaze::DynamicVector<real>& x     // The vector of unknowns
               );

  blaze::DynamicVector<real> r, p, Ap;
};

}

#endif
