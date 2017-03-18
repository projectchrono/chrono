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

#ifndef CHSOLVERBB_H
#define CHSOLVERBB_H

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// An iterative solver based on modified
/// Krylov iteration of spectral projected gradients
/// with Borzilai-Borwein.
/// The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y
///  | Cq -E | |l|  |-b|  |c|
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals)
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0)
/// * case CCP: Y_i are friction cones)

class ChApi ChSolverBB : public ChIterativeSolver {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSolverBB)

  protected:
    int n_armijo;
    int max_armijo_backtrace;
    bool diag_preconditioning;

  public:
    ChSolverBB(int mmax_iters = 50,       ///< max.number of iterations
               bool mwarm_start = false,  ///< uses warm start?
               double mtolerance = 0.0    ///< tolerance for termination criterion
               )
        : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, 0.2) {
        n_armijo = 10;
        max_armijo_backtrace = 3;
        diag_preconditioning = true;
    }

    virtual ~ChSolverBB() {}

    virtual Type GetType() const override { return Type::BARZILAIBORWEIN; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Same as Solve(), but this also supports the presence of
    /// ChKblock blocks. If Solve() is called and stiffness is present,
    /// Solve() automatically falls back to this function.
    /// It does not solve the Schur complement N*l-r=0 as Solve does, here the
    /// entire system KKT matrix with duals l and primals q is used.
    //***TODO*** Solve_SupportingStiffness() was not working. Is there a way to make this working? probably not..
    //***DEPRECATED***
    virtual double Solve_SupportingStiffness(
        ChSystemDescriptor& sysd  ///< system description with constraints and variables
        );

    /// Number of max tolerated steps in non-monotone Armijo
    /// line search; usually good values are in 1..10 range.
    void SetNarmijo(int mf) { this->n_armijo = mf; }
    double GetNarmijo() { return this->n_armijo; }

    void SetMaxArmijoBacktrace(int mm) { this->max_armijo_backtrace = mm; }
    int GetMaxArmijoBacktrace() { return this->max_armijo_backtrace; }

    /// Enable diagonal preconditioning. It a simple but fast
    /// preconditioning technique that is expecially useful to
    /// fix slow convergence in case variables have very different orders
    /// of magnitude.
    void SetDiagonalPreconditioning(bool mp) { this->diag_preconditioning = mp; }
    bool GetDiagonalPreconditioning() { return this->diag_preconditioning; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChSolverBB>();
        // serialize parent class
        ChIterativeSolver::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(n_armijo);
        marchive << CHNVP(max_armijo_backtrace);
        marchive << CHNVP(diag_preconditioning);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChSolverBB>();
        // deserialize parent class
        ChIterativeSolver::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(n_armijo);
        marchive >> CHNVP(max_armijo_backtrace);
        marchive >> CHNVP(diag_preconditioning);
    }
};

}  // end namespace chrono

#endif
