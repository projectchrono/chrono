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
// Implementation of APGD that is more optimized.
// =============================================================================

#ifndef CHSOLVERAPGD_H
#define CHSOLVERAPGD_H

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {

class CH_PARALLEL_API ChSolverAPGD: public ChSolverParallel {
public:
  ChSolverAPGD() : ChSolverParallel() {}
  ~ChSolverAPGD() {}

  void Solve() {
    if (data_container->num_constraints == 0) {
      return;
    }
    data_container->measures.solver.total_iteration +=
      SolveAPGD(max_iteration,
                data_container->num_constraints,
                data_container->host_data.R,
                data_container->host_data.gamma);
  }

  // Solve using a more streamlined but harder to read version of the APGD method
  uint SolveAPGD(const uint max_iter,                    // Maximum number of iterations
                 const uint size,                        // Number of unknowns
                 const blaze::DynamicVector<real>& b,    // Rhs vector
                 blaze::DynamicVector<real>& x           // The vector of unknowns
                 );

  // Compute the residual for the solver
  real Res4(const int SIZE,
            blaze::DynamicVector<real>& mg_tmp2,
            blaze::DynamicVector<real>& x,
            blaze::DynamicVector<real>& mb_tmp);

  // Set parameters for growing and shrinking the step size
  void SetAPGDParams(real theta_k, real shrink, real grow);

  // APGD specific vectors
  blaze::DynamicVector<real> obj2_temp, obj1_temp, ms, mg_tmp2, mb_tmp, mg_tmp, mg_tmp1, mg, mx, my, ml_candidate, mso;
  real L_k, t_k;
  real init_theta_k;
  real step_shrink;
  real step_grow;
  real old_objective;
  real lastgoodres;
  real theta_k;
  real theta_k1;
  real beta_k1;
  real mb_tmp_norm, mg_tmp_norm;
  real obj1, obj2;
  real dot_mg_ms, norm_ms;
  real delta_obj;
};

}

#endif
