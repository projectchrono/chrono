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

#ifndef CHSOLVERBBMULTIDOMAIN_H
#define CHSOLVERBBMULTIDOMAIN_H

#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono_multidomain/ChApiMultiDomain.h"

namespace chrono {
namespace multidomain {

/// @addtogroup chrono_solver
/// @{

/// An iterative solver based on modified Krylov iteration of spectral projected gradients with Barzilai-Borwein.
/// This works in multidomain mode.
/// 
/// The Barzilai-Borwein solver can use diagonal preconditioning (enabled by default).

class ChApiMultiDomain ChSolverBBmultidomain : public ChIterativeSolverVI {
  public:
    ChSolverBBmultidomain();

    ~ChSolverBBmultidomain() {}

    //virtual Type GetType() const override { return Type::BARZILAIBORWEIN; }

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

}  // end namespace multidomain
}  // end namespace chrono

#endif
