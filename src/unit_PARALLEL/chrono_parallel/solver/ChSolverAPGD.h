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

#ifndef CHSOLVERAPGD_H
#define CHSOLVERAPGD_H

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverAPGD : public ChSolverParallel {
 public:
  ChSolverAPGD();
  ~ChSolverAPGD() {}

  void Solve() {
    if (data_container->num_constraints == 0) {
      return;
    }

    data_container->measures.solver.total_iteration += SolveAPGD(max_iteration, data_container->num_constraints, data_container->host_data.R, data_container->host_data.gamma);
  }

  // Solve using a more streamlined but harder to read version of the APGD method
  uint SolveAPGD(const uint max_iter,                    // Maximum number of iterations
                 const uint size,                        // Number of unknowns
                 const blaze::DynamicVector<real>& b,    // Rhs vector
                 blaze::DynamicVector<real>& x           // The vector of unknowns
                 );

  // Compute the residual for the solver
  real Res4(blaze::DynamicVector<real>& mg_tmp2, blaze::DynamicVector<real>& x, blaze::DynamicVector<real>& mb_tmp);

  // APGD specific vectors
  blaze::DynamicVector<real> obj2_temp, obj1_temp, temp, g, gamma_new, y, gamma_hat, N_gamma_new;
  real L, t;
  real theta, theta_new, beta_new;
  real mb_tmp_norm, mg_tmp_norm;
  real obj1, obj2;
  real dot_g_temp, norm_ms;
};
}
#endif
