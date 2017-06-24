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

#ifndef CHSOLVERSOR_H
#define CHSOLVERSOR_H

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// An iterative solver based on projective fixed point method, with overrelaxation
/// and immediate variable update as in SOR methods.\n
/// See ChSystemDescriptor for more information about the problem formulation and the data structures
/// passed to the solver.

class ChApi ChSolverSOR : public ChIterativeSolver {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSolverSOR)

  public:
    ChSolverSOR(int mmax_iters = 50,       ///< max.number of iterations
                bool mwarm_start = false,  ///< uses warm start?
                double mtolerance = 0.0,   ///< tolerance for termination criterion
                double momega = 1.0        ///< overrelaxation criterion
                )
        : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, momega) {}

    virtual ~ChSolverSOR() {}

    virtual Type GetType() const override { return Type::SOR; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;
};

}  // end namespace chrono

#endif
