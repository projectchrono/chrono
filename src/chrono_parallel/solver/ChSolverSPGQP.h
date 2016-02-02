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
// This file contains an implementation of BB that is more optimized.
// =============================================================================

#pragma once

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverSPGQP : public ChSolverParallel {
  public:
    ChSolverSPGQP();
    ~ChSolverSPGQP() {}

    // Solve using a more streamlined but harder to read version of the BB method
    uint Solve(ChShurProduct& ShurProduct,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& b,  // Rhs vector
               DynamicVector<real>& x         // The vector of unknowns
               );

    void UpdateR();

    // BB specific vectors
    real alpha, f_max, xi, beta_bar, beta_tilde, beta_k, gam;
    DynamicVector<real> g, d_k, x, temp, Ad_k, g_alpha, x_candidate;
    std::vector<real> f_hist;
};
}
