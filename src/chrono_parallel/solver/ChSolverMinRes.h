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
// Implementation of an iterative MinRes solver.
// =============================================================================

#pragma once

#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {

class CH_PARALLEL_API ChSolverMinRes : public ChSolverParallel {
  public:
    ChSolverMinRes() : ChSolverParallel() {}
    ~ChSolverMinRes() {}

    // Solve using the minimal residual method
    uint Solve(ChShurProduct& ShurProduct,
               const uint max_iter,     // Maximum number of iterations
               const uint size,         // Number of unknowns
               DynamicVector<real>& b,  // Rhs vector
               DynamicVector<real>& x   // The vector of unknowns
               );

    DynamicVector<real> v, v_hat, w, w_old, xMR, v_old, Av, w_oold;
};
}
