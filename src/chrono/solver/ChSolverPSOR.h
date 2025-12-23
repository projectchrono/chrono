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

#ifndef CHSOLVER_PSOR_H
#define CHSOLVER_PSOR_H

#include "chrono/solver/ChIterativeSolverVI.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// An iterative solver based on projective fixed-point method, with overrelaxation and immediate variable update as in
/// SOR methods.
///
/// Note: this solver can not be used for problems that include stiffness or damping blocks in system descriptor
/// (e.g. coming from FEA)
///
/// See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the
/// solver.

class ChApi ChSolverPSOR : public ChIterativeSolverVI {
  public:
    ChSolverPSOR();

    ~ChSolverPSOR() {}

    virtual Type GetType() const override { return Type::PSOR; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Return the tolerance error reached during the last solve.
    /// For the PSOR solver, this is the maximum constraint violation.
    virtual double GetError() const override { return maxviolation; }

  private:
    double maxviolation;
};

/// @} chrono_solver

}  // end namespace chrono

#endif
