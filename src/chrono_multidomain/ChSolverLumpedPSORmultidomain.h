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

#ifndef CHSOLVER_LUMPEDPSOR_MULTIDOMAIN_H
#define CHSOLVER_LUMPEDPSOR_MULTIDOMAIN_H

#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono_multidomain/ChApiMultiDomain.h"
#include "chrono/solver/ChSolverLumped.h"

namespace chrono {
namespace multidomain {


/// This ChSolverLumpedPSORmultidomain is the multidomain version of ChSolverLumpedPSOR
/// It is an iterative solver based on projective fixed-point method, with overrelaxation and immediate variable update as in
/// SOR methods. A diagonal lumped mass is used.
///
/// Differently from ChSolverPSORmultidomain, here *lumped mass* is used to compute the acceleration, so if there are FEA
/// their mass is considered too (but not their stiffness or damping matrices). This means that if you use this
/// solver with an implicit timestepper, constraints/contacts would behave like in an implicit step, but the
/// FEA would behave like in an explicit Euler step. If you use it in an explicit timestepper,
/// then the FEA would behave like in an explicit step, but the constraints/contacts would be more precise
/// than using just penalty constraints. In implicit timestepping, many PSOR iters might be needed. In explicit
/// timestepping, just one PSOR iteration could be enough, as a post-projection.
/// Typically, one uses this solver together with ChTimestepperExplicitProjected.

class ChApiMultiDomain ChSolverLumpedPSORmultidomain : public ChIterativeSolverVI, public ChSolverLumpedBase {
  public:
    ChSolverLumpedPSORmultidomain();

    ~ChSolverLumpedPSORmultidomain() {}

    //virtual Type GetType() const override { return Type::PSOR; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Return the tolerance error reached during the last solve.
    /// For the PSOR solver, this is the maximum constraint violation.
    virtual double GetError() const override { return maxviolation; }


    /// SetThis to 1 for exchanging solution vector during each iteration of the refinement,
    /// Set this to N for exchanging solution vector each Nth iteration of the refinement, and at the last iteration anyway.
    void SetCommunicationEachIteration(int mi) {
        this->communication_each = mi;
    }

    /// For the nodes at the interfaces this method is similar to a projected Jacobi. Its convergence is a bit worse than
    /// SOR, and it also might require a lower relaxation factor to avoid divergence. This interface-specific omega 
    /// can be set here. Default is 0.25.
    void SetOmegaInterface(double omega) {
        this->omega_interface = omega;
    }

  private:
    double maxviolation;
    int    communication_each;
    double omega_interface;
};



/// @} chrono_solver

}  // end namespace multidomain
}  // end namespace chrono

#endif
