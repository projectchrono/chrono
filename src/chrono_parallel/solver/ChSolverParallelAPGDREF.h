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
// Authors: Hammad Mazhar, Daniel Melanz
// =============================================================================
//
// Implementation of APGD that is exactly like the thesis
// =============================================================================

#pragma once

#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {

class CH_PARALLEL_API ChSolverParallelAPGDREF : public ChSolverParallel {
 public:
   ChSolverParallelAPGDREF() : ChSolverParallel() {}
   ~ChSolverParallelAPGDREF() {}

  void Solve() {
    if (data_manager->num_constraints == 0) {
      return;
    }
    data_manager->system_timer.start("ChSolverParallel_Solve");
    data_manager->measures.solver.total_iteration += SolveAPGDREF(
        max_iteration, data_manager->num_constraints, data_manager->host_data.R, data_manager->host_data.gamma);
    data_manager->system_timer.stop("ChSolverParallel_Solve");
  }

  // Solve using the APGD method
  uint SolveAPGDREF(const uint max_iter,                  // Maximum number of iterations
                    const uint size,                      // Number of unknowns
                    const DynamicVector<real>& r,  // Rhs vector
                    DynamicVector<real>& gamma     // The vector of unknowns
                    );

  // Compute the residual for the solver
  real Res4(DynamicVector<real>& gamma, const DynamicVector<real>& r, DynamicVector<real>& tmp);

  // APGD specific vectors
  DynamicVector<real> gamma_hat;
  DynamicVector<real> gammaNew, g, y, yNew, tmp;
};
}

