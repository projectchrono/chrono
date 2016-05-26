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
// This file contains an implementation of APGD that is more optimized.
// =============================================================================

#pragma once

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverParallelAPGD : public ChSolverParallel {
 public:
  ChSolverParallelAPGD();
  ~ChSolverParallelAPGD() {}

  void Solve() {
    if (data_manager->num_constraints == 0) {
      return;
    }

    data_manager->measures.solver.total_iteration += SolveAPGD(
        max_iteration, data_manager->num_constraints, data_manager->host_data.R, data_manager->host_data.gamma);
  }

  // Solve using a more streamlined but harder to read version of the APGD method
  uint SolveAPGD(const uint max_iter,                  // Maximum number of iterations
                 const uint size,                      // Number of unknowns
                 const DynamicVector<real>& b,  // Rhs vector
                 DynamicVector<real>& x         // The vector of unknowns
                 );

  void UpdateR();

  // APGD specific vectors
  DynamicVector<real> obj2_temp, obj1_temp, temp, g, gamma_new, y, gamma_hat, N_gamma_new;
  real L, t;
  real g_diff;
  real theta, theta_new, beta_new;
  real mb_tmp_norm, mg_tmp_norm;
  real obj1, obj2;
  real dot_g_temp, norm_ms;
};
}
