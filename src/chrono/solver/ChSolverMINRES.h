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

#ifndef CHSOLVERMINRES_H
#define CHSOLVERMINRES_H

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// An iterative solver based on modified Krylov iteration of MINRES type alternated with
/// gradient projection (active set).\n
/// See ChSystemDescriptor for more information about the problem formulation and the data structures
/// passed to the solver.

class ChApi ChSolverMINRES : public ChIterativeSolver {
  protected:
    double feas_tolerance;
    int max_fixedpoint_steps;
    bool diag_preconditioning;
    double rel_tolerance;

  public:
    ChSolverMINRES(int mmax_iters = 50,       ///< max.number of iterations
                   bool mwarm_start = false,  ///< uses warm start?
                   double mtolerance = 0.0    ///< tolerance for termination criterion
    );

    virtual ~ChSolverMINRES() {}

    virtual Type GetType() const override { return Type::MINRES; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Same as Solve(), but this also supports the presence of ChKblock blocks. If Solve() is called and stiffness is
    /// present, Solve() automatically falls back to this function.
    double Solve_SupportingStiffness(ChSystemDescriptor& sysd  ///< system description with constraints and variables
    );

    void SetFeasTolerance(double mf) { this->feas_tolerance = mf; }
    double GetFeasTolerance() { return this->feas_tolerance; }

    void SetMaxFixedpointSteps(int mm) { this->max_fixedpoint_steps = mm; }
    int GetMaxFixedpointSteps() { return this->max_fixedpoint_steps; }

    void SetRelTolerance(double mrt) { this->rel_tolerance = mrt; }
    double GetRelTolerance() { return this->rel_tolerance; }

    void SetDiagonalPreconditioning(bool mp) { this->diag_preconditioning = mp; }
    bool GetDiagonalPreconditioning() { return this->diag_preconditioning; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace chrono

#endif
