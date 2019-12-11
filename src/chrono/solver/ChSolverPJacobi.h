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

#ifndef CHSOLVERJACOBI_H
#define CHSOLVERJACOBI_H

#include "chrono/solver/ChIterativeSolverVI.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// An iterative solver for VI based on projective fixed point method (projected Jacobi). \n
/// Note: this method is here mostly for comparison and tests; we suggest you to use the more efficient ChSolverPSOR.\n
///
/// See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the
/// solver.
class ChApi ChSolverPJacobi : public ChIterativeSolverVI {
  public:
    ChSolverPJacobi();

    ~ChSolverPJacobi() {}

    virtual Type GetType() const override { return Type::PJACOBI; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Return the tolerance error reached during the last solve.
    /// For the PJacobi solver, this is the maximum constraint violation.
    virtual double GetError() const override { return maxviolation; }

  private:
    double maxviolation;
};

/// @} chrono_solver

}  // end namespace chrono

#endif
