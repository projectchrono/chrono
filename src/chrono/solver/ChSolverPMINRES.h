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

#include "chrono/solver/ChIterativeSolverVI.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// An iterative solver based on modified Krylov iteration of MINRES type with gradient projections (similar to
/// nonlinear CG with Polyak-Ribiere).
///
/// The PMINRES solver can use diagonal precondityioning (enabled by default).
///
/// See ChSystemDescriptor for more information about the problem formulation and solver data structures.
///
/// \deprecated Use ChSolverMINRES instead. This class will be removed in a future Chrono release.
class ChApi
/// \cond
CH_DEPRECATED("deprecated. Use ChSolverMINRES instead.")
    /// \endcond
    ChSolverPMINRES : public ChIterativeSolverVI {
  public:
    ChSolverPMINRES();

    ~ChSolverPMINRES() {}

    virtual Type GetType() const override { return Type::PMINRES; }

    /// Perform the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd) override;

    /// Same as Solve(), but this also supports the presence of ChKRMBlock blocks.
    /// A call to Solve() for a system that includes stiffness automatically falls back to this function.
    /// Unlike Solve, which uses the Schur complement to solve N*l-r=0, this function works on the system-wide KKT
    /// matrix with duals l and primals q is used.
    double Solve_SupportingStiffness(ChSystemDescriptor& sysd);

    /// For the case where inequalities are introduced, the gradient is projected.
    /// A numerical differentiation is used, this is the delta.
    void SetGradDiffStep(double mf) { this->grad_diffstep = mf; }
    double GetGradDiffStep() { return this->grad_diffstep; }

    /// Set the relative tolerance.
    /// The iteration stops when the (projected) residual r is smaller than absolute tolerance, set via SetTolerance(),
    /// or it is smaller than 'rhs_rel_tol', i.e. the norm of the right hand side multiplied by this relative tolerance.
    /// Set to 0 to disable relative tolerance checks.
    void SetRelTolerance(double mrt) { this->rel_tolerance = mrt; }
    double GetRelTolerance() { return this->rel_tolerance; }

    /// Return the tolerance error reached during the last solve.
    /// For the PMINRES solver, this is the norm of the projected residual.
    virtual double GetError() const override { return r_proj_resid; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double grad_diffstep;
    double rel_tolerance;
    double r_proj_resid;
};

/// @} chrono_solver

}  // end namespace chrono

#endif
