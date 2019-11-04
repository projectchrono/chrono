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

#ifndef CHSOLVERMUMPS_H
#define CHSOLVERMUMPS_H

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/solver/ChSolver.h"
#include "chrono/solver/ChSystemDescriptor.h"

#include "chrono_mumps/ChApiMumps.h"
#include "chrono_mumps/ChMumpsEngine.h"

namespace chrono {

/// @addtogroup mumps_module
/// @{

/// Interface to the MUMPS parallel direct solver.
class ChApiMumps ChSolverMumps : public ChSolver {
  public:
    enum class MatrixSymmetryType {
        GENERAL,              ///< unsymmetric matrix
        SYMMETRIC_POSDEF,     ///< symmetric positive definite
        SYMMETRIC_INDEF,      ///< symmetric indefinite
        STRUCTURAL_SYMMETRIC  ///< structurally symmetric
    };

    ChSolverMumps();
    ~ChSolverMumps() {}

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
    void SetMatrixSymmetryType(MatrixSymmetryType symmetry) { m_symmetry = symmetry; }

    /// Enable detection of null pivots.
    void EnableNullPivotDetection(bool val, double threshold = 0);

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

    /// Get a handle to the underlying Mumps engine.
    ChMumpsEngine& GetMumpsEngine() { return m_engine; }

    /// Get a handle to the underlying matrix.
    ChSparseMatrix& GetMatrix() { return m_mat; }

    /// Indicate whether or not the #Solve() phase requires an up-to-date problem matrix.
    /// As typical of direct solvers, the MUMPS solver only requires the matrix for its #Setup() phase.
    virtual bool SolveRequiresMatrix() const override { return false; }

    /// Perform the solver setup operations.
    /// For the Mumps solver, this means assembling and factorizing the system matrix.
    /// Returns true if successful and false otherwise.
    virtual bool Setup(ChSystemDescriptor& sysd) override;

    /// Solve using MUMPS.
    /// sysd is the system description with constraints and variables.
    virtual double Solve(ChSystemDescriptor& sysd) override;

  private:
    ChMumpsEngine m_engine;             ///< interface to Mumps solver
    ChSparseMatrix m_mat;               ///< problem matrix
    MatrixSymmetryType m_symmetry;      ///< symmetry of problem matrix
    ChVectorDynamic<double> m_rhs_sol;  ///< right-hand side vector (will be overridden by solution)
    ChVectorDynamic<double> m_rhs_bkp;  ///< solution vector

    int m_dim = 0;      ///< problem size
    double m_sparsity;  ///< user-supplied estimate of matrix sparsity
    int m_solve_call;   ///< counter for calls to Solve
    int m_setup_call;   ///< counter for calls to Setup

    bool m_lock;          ///< is the matrix sparsity pattern locked?
    bool m_use_learner;   ///< use the sparsity pattern learner?
    bool m_force_update;  ///< force a call to the sparsity pattern learner?

    bool m_use_rhs_sparsity;      ///< leverage right-hand side sparsity?
    bool m_null_pivot_detection;  ///< leverage right-hand side sparsity?

    ChTimer<> m_timer_setup_assembly;    ///< timer for matrix assembly
    ChTimer<> m_timer_setup_solvercall;  ///< timer for factorization
    ChTimer<> m_timer_solve_assembly;    ///< timer for RHS assembly
    ChTimer<> m_timer_solve_solvercall;  ///< timer for solution
};

/// @} mumps_module

}  // namespace chrono

#endif
