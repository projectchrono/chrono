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

/// Base class for all solvers that require some form of lumped mass
/// as an approximation or substitute for the full mass matrix. 
/// This class provides a common interface for setting the diagonal mass vector,
/// which is used in the solution process.

class ChApi ChSolverLumpedBase {
  public:
    ChSolverLumpedBase() {}

    ~ChSolverLumpedBase() {}

    /// Diagonal mass. To be set before calling Solve()
    ChVectorDynamic<> diagonal_M;
};


/// A solver for explicit methods, based on constraint regularization 
/// and diagonal mass lumping.

class ChApi ChSolverLumped : public ChSolver, public ChSolverLumpedBase {
public:
    ChSolverLumped() {}

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

//private:

};

/// @} chrono_solver

}  // end namespace chrono

#endif
