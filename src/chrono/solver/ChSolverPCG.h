// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSOLVERPCG_H
#define CHSOLVERPCG_H

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// An iterative solver based on modified Krylov iteration of projected conjugate gradient.\n
/// See ChSystemDescriptor for more information about the problem formulation and the data structures
/// passed to the solver.

class ChApi ChSolverPCG : public ChIterativeSolver {

  public:
    ChSolverPCG(int mmax_iters = 50,       ///< max.number of iterations
                bool mwarm_start = false,  ///< uses warm start?
                double mtolerance = 0.0    ///< tolerance for termination criterion
                )
        : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, 0.2) {}

    virtual ~ChSolverPCG() {}

    virtual Type GetType() const override { return Type::PCG; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;
};

}  // end namespace chrono

#endif
