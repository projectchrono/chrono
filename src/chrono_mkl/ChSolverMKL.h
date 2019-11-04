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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

//// RADU
//// Move implementation to cpp file

#ifndef CHSOLVERMKL_H
#define CHSOLVERMKL_H

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/solver/ChSolver.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono_mkl/ChApiMkl.h"

#define EIGEN_USE_MKL

#include <Eigen/PardisoSupport>

namespace chrono {

/// @addtogroup mkl_module
/// @{

/** \class ChSolverMKL
\brief Interface to the Intel MKL Pardiso parallel direct solver.

Sparse linear direct solver.
Cannot handle VI and complementarity problems, so it cannot be used with NSC formulations.

The solver is equipped with two main features:
- sparsity pattern lock
- sparsity pattern learning

The sparsity pattern \e lock skips sparsity identification or reserving memory for nonzeros on all but the first call to
Setup. This feature is intended for problems where the system matrix sparsity pattern does not change significantly from
call to call.\n
See #LockSparsityPattern();

The sparsity pattern \e learning feature acquires the sparsity pattern in advance, in order to speed up matrix assembly.
Enabled by default, the sparsity matrix learner identifies the exact matrix sparsity pattern (without actually setting
any nonzeros).\n 
See #UseSparsityPatternLearner();

A further option allows the user to provide an estimate for the matrix sparsity (a value in [0,1], with 0 corresponding
to a fully dense matrix). This value is used if the sparsity pattern learner is disabled if/when required to reserve
space for matrix indices and nonzeros.
See #SetSparsityEstimate();

<br>

<div class="ce-warning">
If appropriate and warranted by the problem setup, it is \e highly recommended to enable the sparsity pattern \e lock.
This can significantly improve performance for more complex problems (larger size and/or problems which include
constraints).
</div>

Minimal usage example, to be put anywhere in the code, before starting the main simulation loop:
\code{.cpp}
auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
application.GetSystem()->SetSolver(mkl_solver);
\endcode

See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the solver.
*/
class ChApiMkl ChSolverMKL : public ChSolver {
  public:
    ChSolverMKL();
    ~ChSolverMKL() {}

    /// Enable/disable locking the sparsity pattern (default: false).\n
    /// If enabled, the sparsity pattern of the problem matrix is assumed to be unchanged from call to call.
    /// Enable this option whenever possible to improve performance.
    void LockSparsityPattern(bool val) { m_lock = val; }

    /// Enable/disable use of the sparsity pattern learner (default: enabled).\n
    /// Disable for smaller problems where the overhead may be too large.
    void UseSparsityPatternLearner(bool val) { m_use_learner = val; }

    /// Force a call to the sparsity pattern learner to update sparsity pattern on the underlying matrix.\n
    /// Such a call may be needed in a situation where the sparsity pattern is locked, but a change in the problem size
    /// or structure occurred. This function has no effect if the sparsity pattern learner is disabled.
    void ForceSparsityPatternUpdate() { m_force_update = true; }

    /// Set estimate for matrix sparsity, a value in [0,1], with 0 indicating a fully dense matrix (default: 0.9).\n
    /// Only used if the sparsity pattern learner is disabled.
    void SetSparsityEstimate(double sparsity) { m_sparsity = sparsity; }

    /// Enable/disable use of permutation vector (default: false).
    /// Currently not used.
    void UsePermutationVector(bool val) { m_use_perm = val; }

    /// Enable/disable leveraging sparsity in right-hand side vector (default: false).
    /// Currently not used.
    void LeverageRhsSparsity(bool val) { m_use_rhs_sparsity = val; }

    /// Reset timers for internal phases in Solve and Setup.
    void ResetTimers();

    /// Get cumulative time for assembly operations in Solve phase.
    double GetTimeSolve_Assembly() const { return m_timer_solve_assembly(); }
    /// Get cumulative time for Pardiso calls in Solve phase.
    double GetTimeSolve_SolverCall() const { return m_timer_solve_solvercall(); }
    /// Get cumulative time for assembly operations in Setup phase.
    double GetTimeSetup_Assembly() const { return m_timer_setup_assembly(); }
    /// Get cumulative time for Pardiso calls in Setup phase.
    double GetTimeSetup_SolverCall() const { return m_timer_setup_solvercall(); }

    /// Return the number of calls to the solver's Setup function.
    int GetNumSetupCalls() const { return m_setup_call; }
    /// Return the number of calls to the solver's Setup function.
    int GetNumSolveCalls() const { return m_solve_call; }

    /// Get a handle to the underlying MKL engine.
    Eigen::PardisoLU<ChSparseMatrix>& GetMklEngine() { return m_engine; }

    /// Get a handle to the underlying matrix.
    ChSparseMatrix& GetMatrix() { return m_mat; }

    /// Indicate whether or not the #Solve() phase requires an up-to-date problem matrix.
    /// As typical of direct solvers, the Pardiso solver only requires the matrix for its #Setup() phase.
    virtual bool SolveRequiresMatrix() const override { return false; }

    /// Perform the solver setup operations.
    /// For the MKL solver, this means assembling and factorizing the system matrix.
    /// Returns true if successful and false otherwise.
    virtual bool Setup(ChSystemDescriptor& sysd) override;

    /// Solve using the MKL Pardiso sparse direct solver.
    /// It uses the matrix factorization obtained at the last call to Setup().
    virtual double Solve(ChSystemDescriptor& sysd) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    Eigen::PardisoLU<ChSparseMatrix> m_engine;  ///< underlying Eigen Pardiso interface
    ChSparseMatrix m_mat;                       ///< problem matrix
    ChVectorDynamic<double> m_rhs;              ///< right-hand side vector
    ChVectorDynamic<double> m_sol;              ///< solution vector

    int m_dim;          ///< problem size
    double m_sparsity;  ///< user-supplied estimate of matrix sparsity
    int m_solve_call;   ///< counter for calls to Solve
    int m_setup_call;   ///< counter for calls to Setup

    bool m_lock;          ///< is the matrix sparsity pattern locked?
    bool m_use_learner;   ///< use the sparsity pattern learner?
    bool m_force_update;  ///< force a call to the sparsity pattern learner?

    bool m_use_perm;          ///< use of the permutation vector?
    bool m_use_rhs_sparsity;  ///< leverage right-hand side sparsity?

    ChTimer<> m_timer_setup_assembly;    ///< timer for matrix assembly
    ChTimer<> m_timer_setup_solvercall;  ///< timer for factorization
    ChTimer<> m_timer_solve_assembly;    ///< timer for RHS assembly
    ChTimer<> m_timer_solve_solvercall;  ///< timer for solution

    void GetErrorMessage(Eigen::ComputationInfo error) const;
};

/// @} mkl_module

}  // end namespace chrono

#endif
