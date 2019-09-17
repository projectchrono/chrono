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

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// An iterative solver based on modified
/// Krylov iteration of spectral projected gradients
/// with Barzilai-Borwein.\n
/// See ChSystemDescriptor for more information about the problem formulation and the data structures
/// passed to the solver.

class ChApi ChSolverBB : public ChIterativeSolver {
  protected:
    int n_armijo;
    int max_armijo_backtrace;
    bool diag_preconditioning;

  public:
    ChSolverBB(int mmax_iters = 50,       ///< max.number of iterations
               bool mwarm_start = false,  ///< uses warm start?
               double mtolerance = 0.0    ///< tolerance for termination criterion
    );

    virtual ~ChSolverBB() {}

    virtual Type GetType() const override { return Type::BARZILAIBORWEIN; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Number of max tolerated steps in non-monotone Armijo
    /// line search; usually good values are in 1..10 range.
    void SetNarmijo(int mf) { this->n_armijo = mf; }
    double GetNarmijo() { return this->n_armijo; }

    void SetMaxArmijoBacktrace(int mm) { this->max_armijo_backtrace = mm; }
    int GetMaxArmijoBacktrace() { return this->max_armijo_backtrace; }

    /// Enable diagonal preconditioning. It a simple but fast
    /// preconditioning technique that is especially useful to
    /// fix slow convergence in case variables have very different orders
    /// of magnitude.
    void SetDiagonalPreconditioning(bool mp) { this->diag_preconditioning = mp; }
    bool GetDiagonalPreconditioning() { return this->diag_preconditioning; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif
