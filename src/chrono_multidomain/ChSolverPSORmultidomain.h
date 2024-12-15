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

#ifndef CHSOLVER_PSOR_MULTIDOMAIN_H
#define CHSOLVER_PSOR_MULTIDOMAIN_H

#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono_multidomain/ChApiMultiDomain.h"

namespace chrono {
namespace multidomain {


/// Projected SOR (PSOR) for multidomain.
/// An iterative solver based on projective fixed point method, with overrelaxation and immediate variable update as in
/// SOR methods. At each iteration is shares data with multidomain domain decomposition. \n
/// Inside domains, this works like projected SOR, hence sequentially applying constraints, but at the 
/// coarse level at each iteration the solution is exchanged between all domains at once at the interfaces,
/// so for the interfaces it is similar to a projected Jacobi. \n
/// See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the
/// solver.

class ChApiMultiDomain ChSolverPSORmultidomain : public ChIterativeSolverVI {
  public:
    ChSolverPSORmultidomain();

    ~ChSolverPSORmultidomain() {}

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
