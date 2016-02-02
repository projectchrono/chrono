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

class CH_PARALLEL_API ChSolverAPGDREF : public ChSolverParallel {
  public:
    ChSolverAPGDREF() : ChSolverParallel() {}
    ~ChSolverAPGDREF() {}

    // Solve using the APGD method
    uint Solve(ChShurProduct& ShurProduct,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& r,  // Rhs vector
               DynamicVector<real>& gamma     // The vector of unknowns
               );

    // Compute the residual for the solver
    real Res4(ChShurProduct& ShurProduct,
              DynamicVector<real>& gamma,
              const DynamicVector<real>& r,
              DynamicVector<real>& tmp);

    // APGD specific vectors
    DynamicVector<real> gamma_hat;
    DynamicVector<real> gammaNew, g, y, yNew, tmp;
};
}
