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
// Authors: Radu Serban
// =============================================================================

#ifndef CHSOLVERDIRECT_H
#define CHSOLVERDIRECT_H

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/solver/ChSolver.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// Base class for sparse direct solvers.
class ChApi ChSolverDirect : public ChSolver {
  public:
    enum class MatrixSymmetryType {
        GENERAL,              ///< unsymmetric matrix
        SYMMETRIC_POSDEF,     ///< symmetric positive definite
        SYMMETRIC_INDEF,      ///< symmetric indefinite
        STRUCTURAL_SYMMETRIC  ///< structurally symmetric
    };

    virtual ~ChSolverDirect() {}

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

    /// Set the matrix symmetry type (default: GENERAL).
    virtual void SetMatrixSymmetryType(MatrixSymmetryType symmetry) { m_symmetry = symmetry; }

    /// Enable/disable use of permutation vector (default: false).
    /// A concrete direct sparse solver may or may not support this feature.
    void UsePermutationVector(bool val) { m_use_perm = val; }

    /// Enable/disable leveraging sparsity in right-hand side vector (default: false).
    /// A concrete direct sparse solver may or may not support this feature.
    void LeverageRhsSparsity(bool val) { m_use_rhs_sparsity = val; }

    /// Enable detection of null pivots.
    /// A concrete direct sparse solver may or may not support this feature.
    virtual void EnableNullPivotDetection(bool val, double threshold = 0) { m_null_pivot_detection = val; }

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

    /// Get a handle to the underlying matrix.
    ChSparseMatrix& GetMatrix() { return m_mat; }

    /// Perform the solver setup operations.
    /// Here, sysd is the system description with constraints and variables.
    /// Returns true if successful and false otherwise.
    virtual bool Setup(ChSystemDescriptor& sysd) override;

    /// Solve linear system.
    /// Here, sysd is the system description with constraints and variables.
    virtual double Solve(ChSystemDescriptor& sysd) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    ChSolverDirect();

    /// Factorize the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() = 0;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem() = 0;

    /// Display an error message corresponding to the last failure.
    /// This function is only called if Factorize or Solve returned false.
    virtual void PrintErrorMessage() = 0;

    /// Indicate whether or not the #Solve() phase requires an up-to-date problem matrix.
    /// Typically, direct solvers only require the matrix for their #Setup() phase.
    virtual bool SolveRequiresMatrix() const override { return false; }

    ChSparseMatrix m_mat;           ///< problem matrix
    int m_dim;                      ///< problem size
    MatrixSymmetryType m_symmetry;  ///< symmetry of problem matrix
    double m_sparsity;              ///< user-supplied estimate of matrix sparsity
    ChVectorDynamic<double> m_rhs;  ///< right-hand side vector
    ChVectorDynamic<double> m_sol;  ///< solution vector

    int m_solve_call;  ///< counter for calls to Solve
    int m_setup_call;  ///< counter for calls to Setup

    bool m_lock;          ///< is the matrix sparsity pattern locked?
    bool m_use_learner;   ///< use the sparsity pattern learner?
    bool m_force_update;  ///< force a call to the sparsity pattern learner?

    bool m_use_perm;              ///< use of the permutation vector?
    bool m_use_rhs_sparsity;      ///< leverage right-hand side sparsity?
    bool m_null_pivot_detection;  ///< enable detection of zero pivots?

    ChTimer<> m_timer_setup_assembly;    ///< timer for matrix assembly
    ChTimer<> m_timer_setup_solvercall;  ///< timer for factorization
    ChTimer<> m_timer_solve_assembly;    ///< timer for RHS assembly
    ChTimer<> m_timer_solve_solvercall;  ///< timer for solution
};

/// @} chrono_solver

}  // end namespace chrono

#endif