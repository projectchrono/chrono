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

#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>

namespace chrono {

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
*/
class ChApi ChIterativeSolverLS : public ChSolverLS {
  public:
    virtual ~ChIterativeSolverLS();

    /// Set the maximum number of iterations.\n
    /// Default is twice the matrix size.
    void SetMaxIterations(int max_iterations) { m_max_iterations = max_iterations; }

    /// Set the tolerance threshold used by the stopping criteria.\n
    /// This value is used as an upper bound to the relative residual error: |Ax - b|/|b|. \n
    /// The default value is the machine precision.
    void SetTolerance(double tolerance) { m_tolerance = tolerance; }

    /// Enable/disable use of a simple diagonal preconditioner (default: true).
    void EnableDiagonalPreconditioner(bool val) { m_use_precond = val; }

    /// Enable/disable warm starting by providing an initial guess (default: false).\n
    /// If enabled, the solvers use as an initial guess the current values for [x; -lambda].\n
    /// ATTENTION: enable this option *only* if using the Euler implicit linearized integrator!
    void EnableWarmStart(bool val) { m_warm_start = val; }

    /// Return the number of iterations performed during the last solve.
    virtual int GetIterations() const = 0;

    /// Return the tolerance error reached during the last solve.\n
    /// It is a close approximation of the true relative residual error |Ax-b|/|b|.
    virtual double GetError() const = 0;

    /// Perform the solver setup operations.\n
    /// Here, sysd is the system description with constraints and variables.
    /// Returns true if successful and false otherwise.
    virtual bool Setup(ChSystemDescriptor& sysd) override;

    /// Performs the solution of the problem.\n
    /// Return the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd) override;

  protected:
    ChIterativeSolverLS();

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

    bool m_use_precond;    ///< use diagonal preconditioning?
    bool m_warm_start;     ///< use initial guesss?
    int m_max_iterations;  ///< maximum number of iterations
    double m_tolerance;    ///< tolerance threshold in stopping criteria

  private:
    /// Indicate whether or not the #Solve() phase requires an up-to-date problem matrix.
    virtual bool SolveRequiresMatrix() const override final { return true; }
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

}  // end namespace chrono

#endif
