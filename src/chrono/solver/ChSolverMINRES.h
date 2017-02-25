// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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
/// gradient projection (active set).
/// The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, c \in Ny, normal cone to Y
///  | Cq -E | |l|  |-b|  |c|
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones

class ChApi ChSolverMINRES : public ChIterativeSolver {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSolverMINRES)

  protected:
    double feas_tolerance;
    int max_fixedpoint_steps;
    bool diag_preconditioning;
    double rel_tolerance;

  public:
    ChSolverMINRES(int mmax_iters = 50,       ///< max.number of iterations
                   bool mwarm_start = false,  ///< uses warm start?
                   double mtolerance = 0.0    ///< tolerance for termination criterion
                   )
        : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, 0.2) {
        rel_tolerance = 0.0;
        feas_tolerance = 0.2;
        max_fixedpoint_steps = 6;
        diag_preconditioning = true;
    }

    virtual ~ChSolverMINRES() {}

    virtual Type GetType() const override { return Type::MINRES; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Same as Solve(), but this also supports the presence of
    /// ChKblock blocks. If Solve() is called and stiffness is present,
    /// Solve() automatically falls back to this function.
    virtual double Solve_SupportingStiffness(
        ChSystemDescriptor& sysd  ///< system description with constraints and variables
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
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChSolverMINRES>();
        // serialize parent class
        ChIterativeSolver::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(feas_tolerance);
        marchive << CHNVP(max_fixedpoint_steps);
        marchive << CHNVP(diag_preconditioning);
        marchive << CHNVP(rel_tolerance);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChSolverMINRES>();
        // deserialize parent class
        ChIterativeSolver::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(feas_tolerance);
        marchive >> CHNVP(max_fixedpoint_steps);
        marchive >> CHNVP(diag_preconditioning);
        marchive >> CHNVP(rel_tolerance);
    }
};

}  // end namespace chrono

#endif
