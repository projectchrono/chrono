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

#ifndef CHSOLVER_LUMPED_MULTIDOMAIN_H
#define CHSOLVER_LUMPED_MULTIDOMAIN_H

#include "chrono/solver/ChSolver.h"
#include "chrono_multidomain/ChApiMultiDomain.h"

namespace chrono {
namespace multidomain {


/// A solver for explicit methods, based on constraint regularization and diagonal mass lumping.
/// This is the easiest to map on MPI distributed memory architectures.
/// Assumes masses are lumped and available as stencil (clipped) format, not in additive format.

class ChApiMultiDomain ChSolverLumpedMultidomain : public ChSolver {
public:
    ChSolverLumpedMultidomain();

    ~ChSolverLumpedMultidomain() {}

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

    double C_penalty_k = 100.0;
    double C_penalty_r = 1.0;

//private:

};

/// @} chrono_solver

}  // end namespace multidomain
}  // end namespace chrono

#endif
