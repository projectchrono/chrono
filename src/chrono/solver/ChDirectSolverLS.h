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

#ifndef CH_DIRECTSOLVER_LS_H
#define CH_DIRECTSOLVER_LS_H

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/solver/ChSolverLS.h"

#include <Eigen/SparseLU>

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/** \class ChDirectSolverLS
\brief Base class for sparse direct linear solvers.

Sparse linear direct solvers.
Cannot handle VI and complementarity problems, so it cannot be used with NSC formulations.

This base class manages the setup and solution of sparse linear systems in Chrono and defers to derived classes for
performing the actual matrix factorization and then computing the solution vector.
See ChSolverMkl (which implements Eigen's interface to the Intel MKL Pardiso solver) and ChSolverMumps (which interfaces
to the MUMPS solver).

ChDirectSolverLS manages the detection and update of the matrix sparsity pattern, providing two main features:
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

See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the solver.
*/

class ChApi ChDirectSolverLS : public ChSolverLS {
  public:
    enum class MatrixSymmetryType {
        GENERAL,              ///< unsymmetric matrix
        SYMMETRIC_POSDEF,     ///< symmetric positive definite
        SYMMETRIC_INDEF,      ///< symmetric indefinite
        STRUCTURAL_SYMMETRIC  ///< structurally symmetric
    };

    virtual ~ChDirectSolverLS() {}

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

    /// Get shortcut handle to underlying A matrix, for A*x=b
    ChSparseMatrix& A() { return m_mat; }

    /// Get shortcut handle to underlying x solution vector, for A*x=b
    ChVectorDynamic<double>& x() { return m_sol; }

    /// Get shortcut handle to underlying b right hand-side known vector, for A*x=b
    ChVectorDynamic<double>& b() { return m_rhs; }

    /// Perform the solver setup operations.
    /// Here, sysd is the system description with constraints and variables.
    /// Returns true if successful and false otherwise.
    virtual bool Setup(ChSystemDescriptor& sysd) override;

    /// Solve linear system.
    /// Here, sysd is the system description with constraints and variables.
    virtual double Solve(ChSystemDescriptor& sysd) override;

    /// Generic setup-solve without passing through the ChSystemDescriptor,
    /// in cases where a sparse matrix has been already assembled.
    /// Performs the solver setup operations, assuming someone
    /// has already filled A() matrix before calling this.
    virtual bool SetupCurrent();

    /// Generic setup-solve without passing through the ChSystemDescriptor,
    /// in cases where the a sparse matrix has been already assembled.
    /// Here, sysd is the system description with constraints and variables.
    /// Performs the solver setup operations, assuming someone
    /// has already filled the b() vector before calling this.
    /// Call x() afterward to get results.
    virtual double SolveCurrent();

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    ChDirectSolverLS();

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

    ChTimer m_timer_setup_assembly;    ///< timer for matrix assembly
    ChTimer m_timer_setup_solvercall;  ///< timer for factorization
    ChTimer m_timer_solve_assembly;    ///< timer for RHS assembly
    ChTimer m_timer_solve_solvercall;  ///< timer for solution

  private:
    void WriteMatrix(const std::string& filename, const ChSparseMatrix& M);
    void WriteVector(const std::string& filename, const ChVectorDynamic<double>& v);
};

// ---------------------------------------------------------------------------

/// Sparse LU direct solver.\n
/// Interface to Eigen's SparseLU solver, a supernodal LU factorization for general matrices.\n
/// Cannot handle VI and complementarity problems, so it cannot be used with NSC formulations.\n
/// See ChDirectSolverLS for more details.
class ChApi ChSolverSparseLU : public ChDirectSolverLS {
  public:
    ChSolverSparseLU() {}
    ~ChSolverSparseLU() {}
    virtual Type GetType() const override { return Type::SPARSE_LU; }

  private:
    /// Factorize the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() override;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem() override;

    /// Display an error message corresponding to the last failure.
    /// This function is only called if Factorize or Solve returned false.
    virtual void PrintErrorMessage() override;

    Eigen::SparseLU<ChSparseMatrix, Eigen::COLAMDOrdering<int>> m_engine;  ///< Eigen SparseLU solver
};

/// Sparse QR direct solver.\n
/// Interface to Eigen's SparseQR solver, a left-looking rank-revealing QR factorization.\n
/// Cannot handle VI and complementarity problems, so it cannot be used with NSC formulations.\n
/// See ChDirectSolverLS for more details.
class ChApi ChSolverSparseQR : public ChDirectSolverLS {
  public:
    ChSolverSparseQR() {}
    ~ChSolverSparseQR() {}
    virtual Type GetType() const override { return Type::SPARSE_QR; }

  private:
    /// Factorize the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() override;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem() override;

    /// Display an error message corresponding to the last failure.
    /// This function is only called if Factorize or Solve returned false.
    virtual void PrintErrorMessage() override;

    Eigen::SparseQR<ChSparseMatrix, Eigen::COLAMDOrdering<int>> m_engine;  ///< Eigen SparseQR solver
};

/// @} chrono_solver

}  // end namespace chrono

#endif
