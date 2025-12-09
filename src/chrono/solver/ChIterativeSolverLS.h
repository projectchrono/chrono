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
//
// Chrono solvers based on Eigen iterative linear solvers.
// All iterative linear solvers are implemented in a matrix-free context and
// rely on the system descriptor for the required SPMV operations.
// They can optionally use a diagonal preconditioner.
//
// Available solvers:
//   GMRES
//   BiCGSTAB
//   MINRES
//
// =============================================================================

#ifndef CH_ITERATIVESOLVER_LS_H
#define CH_ITERATIVESOLVER_LS_H

#include "chrono/solver/ChSolverLS.h"
#include "chrono/solver/ChIterativeSolver.h"

#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>

namespace chrono {

/// @addtogroup chrono_solver
/// @{

// ---------------------------------------------------------------------------

// Forward declarations of wrapper class for SPMV operations and custom preconditioner
class ChMatrixSPMV;
class ChDiagonalPreconditioner;

// ---------------------------------------------------------------------------

/** \class ChIterativeSolverLS
\brief Base class for Chrono solvers based on Eigen iterative linear solvers.

Iterative linear solvers.\n
Cannot handle VI and complementarity problems, so they cannot be used with NSC formulations.

All iterative solvers are implemented in a matrix-free context and rely on the system descriptor for the required
SPMV operations. See ChSystemDescriptor for more information about the problem formulation and the data structures
passed to the solver.

The default value for the maximum number of iterations is twice the matrix size.

The threshold value specified through #SetTolerance is used by the stopping criteria as an upper bound to the relative
residual error: |Ax - b|/|b|. Default: machine precision.

By default, these solvers use a diagonal preconditioner and no warm start. Recall that the warm start option should
be used **only** in conjunction with the Euler implicit linearized integrator.
*/
class ChApi ChIterativeSolverLS : public ChIterativeSolver, public ChSolverLS {
  public:
    virtual ~ChIterativeSolverLS();

    /// Perform the solver setup operations.
    /// The system descriptor contains the constraints and variables.
    /// The argument `analyze` indicates if a full analysis of the system matrix is required. This is true when a
    /// structural change in the system was detected (e.g., when a physical component was added to or removed from the
    /// Chrono system).
    virtual bool Setup(ChSystemDescriptor& sysd, bool analyze) override;

    /// Perform the solution of the problem.
    /// Returns the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd) override;

  protected:
    ChIterativeSolverLS();

    virtual bool IsIterative() const override { return true; }
    virtual bool IsDirect() const override { return false; }
    virtual ChIterativeSolver* AsIterative() override { return this; }

    /// Indicate whether or not the #Solve() phase requires an up-to-date problem matrix.
    virtual bool SolveRequiresMatrix() const override final { return true; }

    /// Initialize the solver with the current sparse matrix and return true if successful.
    virtual bool SetupProblem() = 0;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveProblem() = 0;

    ChMatrixSPMV* m_spmv;                 ///< matrix-like wrapper for SPMV operations
    ChVectorDynamic<double> m_sol;        ///< solution vector
    ChVectorDynamic<double> m_rhs;        ///< right-hand side vector
    ChVectorDynamic<double> m_invdiag;    ///< inverse diagonal entries (for preconditioning)
    ChVectorDynamic<double> m_initguess;  ///< initial guess (for warm start)
};

// ---------------------------------------------------------------------------

/// GMRES iterative solver.\n
/// Solves Ax=b for general sparse matrix a, using the Generalized Minimal Residual Algorithm based on the Arnoldi
/// algorithm implemented with Householder reflections.\n
/// See ChIterativeSolverLS for supported solver settings and paramters.
class ChApi ChSolverGMRES : public ChIterativeSolverLS {
  public:
    ChSolverGMRES();
    ~ChSolverGMRES();
    virtual Type GetType() const override { return Type::GMRES; }
    virtual int GetIterations() const override;
    virtual double GetError() const override;

  private:
    virtual bool SetupProblem() override;
    virtual bool SolveProblem() override;

    Eigen::GMRES<ChMatrixSPMV, ChDiagonalPreconditioner>* m_engine;
};

// ---------------------------------------------------------------------------

/// BiCGSTAB iterative solver.\n
/// Solves for Ax = b sparse linear problems using a bi-conjugate gradient stabilized algorithm.\n
/// See ChIterativeSolverLS for supported solver settings and paramters.
class ChApi ChSolverBiCGSTAB : public ChIterativeSolverLS {
  public:
    ChSolverBiCGSTAB();
    ~ChSolverBiCGSTAB();
    virtual Type GetType() const override { return Type::BICGSTAB; }
    virtual int GetIterations() const override;
    virtual double GetError() const override;

  private:
    virtual bool SetupProblem() override;
    virtual bool SolveProblem() override;

    Eigen::BiCGSTAB<ChMatrixSPMV, ChDiagonalPreconditioner>* m_engine;
};

// ---------------------------------------------------------------------------

/// MINRES iterative solver.
/// Solves Ax=b for symmetric sparse matrix A, using a conjugate-gradient type method based on Lanczos
/// tridiagonalization.\n
/// See ChIterativeSolverLS for supported solver settings and paramters.
class ChApi ChSolverMINRES : public ChIterativeSolverLS {
  public:
    ChSolverMINRES();
    ~ChSolverMINRES();
    virtual Type GetType() const override { return Type::MINRES; }
    virtual int GetIterations() const override;
    virtual double GetError() const override;

  private:
    virtual bool SetupProblem() override;
    virtual bool SolveProblem() override;

    Eigen::MINRES<ChMatrixSPMV, Eigen::Lower | Eigen::Upper, ChDiagonalPreconditioner>* m_engine;
};

/// @} chrono_solver

}  // end namespace chrono

#endif
