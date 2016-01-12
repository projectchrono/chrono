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
class CH_PARALLEL_API ChSolverBB : public ChSolverParallel {
  public:
    ChSolverBB();
    ~ChSolverBB() {}

    void Solve() {
        if (data_manager->num_constraints == 0) {
            return;
        }

        data_manager->measures.solver.total_iteration += SolveBB(
            max_iteration, data_manager->num_constraints, data_manager->host_data.R, data_manager->host_data.gamma);
    }

    // Solve using a more streamlined but harder to read version of the BB method
    uint SolveBB(const uint max_iter,           // Maximum number of iterations
                 const uint size,               // Number of unknowns
                 const DynamicVector<real>& b,  // Rhs vector
                 DynamicVector<real>& x         // The vector of unknowns
                 );

    void UpdateR();

    // BB specific vectors
    DynamicVector<real> temp, ml, mg, mg_p, ml_candidate, ms, my, mdir, ml_p;
    DynamicVector<real> mD, invmD;
};
}
