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

#ifndef CHSOLVERBB_H
#define CHSOLVERBB_H

#include "chrono/solver/ChIterativeSolverVI.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// An iterative solver based on modified Krylov iteration of Spectral Projected Gradients with Barzilai-Borwein.
///
/// Notes:
/// - can not solve problems including stiffness or damping blocks in system descriptor (e.g. coming from FEA)
/// - diagonal preconditioning is available (enabled by default)
/// 
/// See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the
/// solver.
class ChApi ChSolverBB : public ChIterativeSolverVI {
  public:
    ChSolverBB();

    ~ChSolverBB() {}

    virtual Type GetType() const override { return Type::BARZILAIBORWEIN; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Number of max tolerated steps in non-monotone Armijo
    /// line search; usually good values are in 1..10 range.
    void SetMaxStepsArmijoLineSearch(int mf) { n_armijo = mf; }
    double GetMaxStepsArmijoLineSearch() { return n_armijo; }

    void SetMaxStepsArmijoBacktrace(int mm) { max_armijo_backtrace = mm; }
    int GetMaxStepsArmijoBacktrace() { return max_armijo_backtrace; }

    /// Return the tolerance error reached during the last solve.
    /// For the Barzilai-Borwein solver, this is the norm of the projected gradient.
    virtual double GetError() const override { return lastgoodres; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    int n_armijo;
    int max_armijo_backtrace;
    double lastgoodres;
};

/// @} chrono_solver

}  // end namespace chrono

#endif
