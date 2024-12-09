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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHSOLVER_LUMPED_H
#define CHSOLVER_LUMPED_H

#include "chrono/solver/ChSolver.h"

namespace chrono {


/// A solver for explicit methods, based on constraint regularization 
/// and diagonal mass lumping.

class ChApi ChSolverLumped : public ChSolver {
public:
    ChSolverLumped();

    ~ChSolverLumped() {}

    virtual bool IsIterative() const {
        return false;
    }
    virtual bool IsDirect() const {
        return false;
    }
    virtual bool SolveRequiresMatrix() const {
        return false;
    }


    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
    ) override;

    /// Diagonal mass. To be set before calling Solve()
    ChVectorDynamic<> diagonal_M;

//private:

};

/// @} chrono_solver

}  // end namespace chrono

#endif
