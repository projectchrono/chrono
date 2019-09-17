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

#ifndef CHSOLVERPMINRES_H
#define CHSOLVERPMINRES_H

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// An iterative solver based on modified Krylov iteration of MINRES type with gradient
/// projections (similar to nonlinear CG with Polyak-Ribiere).\n
/// See ChSystemDescriptor for more information about the problem formulation and the data structures
/// passed to the solver.

class ChApi ChSolverPMINRES : public ChIterativeSolver {
  protected:
    double grad_diffstep;
    double rel_tolerance;
    bool diag_preconditioning;

  public:
    ChSolverPMINRES(int mmax_iters = 50,       ///< max.number of iterations
                    bool mwarm_start = false,  ///< uses warm start?
                    double mtolerance = 0.0    ///< tolerance for termination criterion
    );

    virtual ~ChSolverPMINRES() {}

    virtual Type GetType() const override { return Type::PMINRES; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Same as Solve(), but this also supports the presence of
    /// ChKblock blocks. If Solve() is called and stiffness is present,
    /// Solve() automatically falls back to this function.
    /// It does not solve the Schur complement N*l-r=0 as Solve does, here the
    /// entire system KKT matrix with duals l and primals q is used.
    double Solve_SupportingStiffness(ChSystemDescriptor& sysd  ///< system description with constraints and variables
    );

    /// For the case where inequalities are introduced, the
    /// gradient is projected. A numerical differentiation is used, this is the delta.
    void SetGradDiffStep(double mf) { this->grad_diffstep = mf; }
    double GetGradDiffStep() { return this->grad_diffstep; }

    /// Set relative tolerance. the iteration stops when
    /// the (projected) residual r is smaller than absolute tolerance,
    /// that you set via SetTolerance(), OR it is smaller than 'rhs_rel_tol' i.e. the norm
    /// of the right hand side multiplied by this relative tolerance.
    /// Set to 0 if you do not want relative tolerance to enter into play.
    void SetRelTolerance(double mrt) { this->rel_tolerance = mrt; }
    double GetRelTolerance() { return this->rel_tolerance; }

    /// Enable diagonal preconditioning. It a simple but fast
    /// preconditioning technique that is especially useful to
    /// fix slow convergence in case variables have very different orders
    /// of magnitude.
    void SetDiagonalPreconditioning(bool mp) { this->diag_preconditioning = mp; }
    bool GetDiagonalPreconditioning() { return this->diag_preconditioning; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif
