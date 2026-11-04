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

#ifndef CHSOLVER_LUMPEDPSOR_H
#define CHSOLVER_LUMPEDPSOR_H

#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChSolverLumped.h"


namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// An iterative solver based on projective fixed-point method, with overrelaxation and immediate variable update as in
/// SOR methods. A diagonal lumped mass is used.
///
/// Differently from ChSolverPSOR, here a lumped mass is used to compute the acceleration, so if there are FEA
/// their mass is considered (but not their stiffness or damping matrices). This means that if you use this
/// solver with an implicit timestepper, constraints/contacts would behave like in an implicit step, but the 
/// FEA would behave like in an explicit Euler step. If you use it in an explicit timestepper, 
/// then the FEA would behave like in an explicit step, but the constraints/contacts would be more precise 
/// than using just penalty constraints. In implicit timestepping, many PSOR iters might be needed. In explicit
/// timestepping, just one PSOR iteration could be enough, as a post-projection. 
/// Typically, one uses this solver together with ChTimestepperExplicitProjected.
///
/// See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the
/// solver.

class ChApi ChSolverLumpedPSOR : public ChIterativeSolverVI, public ChSolverLumpedBase {
  public:
    ChSolverLumpedPSOR();

    ~ChSolverLumpedPSOR() {}

    virtual Type GetType() const override { return Type::CUSTOM; }

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
