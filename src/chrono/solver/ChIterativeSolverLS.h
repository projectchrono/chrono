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
//
// =============================================================================

#ifndef CH_ITERATIVESOLVER_LS_H
#define CH_ITERATIVESOLVER_LS_H

#include "chrono/solver/ChSolverLS.h"

#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>

namespace chrono {

// Forward declaration of wrapper class for SPMV operations
class ChMatrixSPMV;

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

    /// Set the maximum number of iterations. \n
    /// Default is twice the matrix size.
    void SetMaxIterations(int max_iterations) { m_max_iterations = max_iterations; }

    /// Set the tolerance threshold used by the stopping criteria. \n
    /// This value is used as an upper bound to the relative residual error: |Ax - b|/|b|. \n
    /// The default value is the machine precision.
    void SetTolerance(double tolerance) { m_tolerance = tolerance; }

    /// Return the number of iterations performed during the last solve.
    virtual int GetIterations() const = 0;

    /// Return the tolerance error reached during the last solve. \n
    /// It is a close approximation of the true relative residual error |Ax-b|/|b|.
    virtual double GetError() const = 0;

    /// Perform the solver setup operations.
    /// Here, sysd is the system description with constraints and variables.
    /// Returns true if successful and false otherwise.
    virtual bool Setup(ChSystemDescriptor& sysd) override;

    /// Performs the solution of the problem.
    /// Return the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd) override;

  protected:
    ChIterativeSolverLS();

    /// Initialize the solver with the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() = 0;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem() = 0;

    ChMatrixSPMV* m_spmv;
    ChVectorDynamic<double> m_sol;  ///< solution vector
    ChVectorDynamic<double> m_rhs;  ///< right-hand side vector

    int m_max_iterations;  ///< maximum number of iterations
    double m_tolerance;    ///< tolerance threshold in stopping criteria

  private:
    /// Indicate whether or not the #Solve() phase requires an up-to-date problem matrix.
    virtual bool SolveRequiresMatrix() const override final { return false; }
};

/// GMRES iterative solver.
/// Solves Ax=b for general sparse matrix a, using the Generalized Minimal Residual Algorithm based on the Arnoldi
/// algorithm implemented with Householder reflections.
class ChApi ChSolverGMRES : public ChIterativeSolverLS {
  public:
    ChSolverGMRES();
    ~ChSolverGMRES();
    virtual Type GetType() const override { return Type::GMRES; }
    virtual int GetIterations() const override;
    virtual double GetError() const override;

  private:
    virtual bool FactorizeMatrix() override;
    virtual bool SolveSystem() override;

    Eigen::GMRES<ChMatrixSPMV, Eigen::IdentityPreconditioner>* m_engine;
    ////Eigen::GMRES<ChMatrixSPMV, Eigen::DiagonalPreconditioner<double>>* m_engine;
};

/// BiCGSTAB iterative solver.
/// Solves for Ax = b sparse linear problems using a bi conjugate gradient stabilized algorithm. 
class ChApi ChSolverBiCGSTAB : public ChIterativeSolverLS {
  public:
    ChSolverBiCGSTAB();
    ~ChSolverBiCGSTAB();
    virtual Type GetType() const override { return Type::BICGSTAB; }
    virtual int GetIterations() const override;
    virtual double GetError() const override;

  private:
    virtual bool FactorizeMatrix() override;
    virtual bool SolveSystem() override;

    Eigen::BiCGSTAB<ChMatrixSPMV, Eigen::IdentityPreconditioner>* m_engine;
    ////Eigen::BiCGSTAB<ChMatrixSPMV, Eigen::DiagonalPreconditioner<double>>* m_engine;
};


/// MINRES iterative solver.
/// Solves Ax=b for symmetric sparse matrix A, using a conjugate-gradient type method based on Lanczos
/// tridiagonalization.
class ChApi ChSolverMINRES : public ChIterativeSolverLS {
  public:
    ChSolverMINRES();
    ~ChSolverMINRES();
    virtual Type GetType() const override { return Type::MINRES; }
    virtual int GetIterations() const override;
    virtual double GetError() const override;

  private:
    virtual bool FactorizeMatrix() override;
    virtual bool SolveSystem() override;

    Eigen::MINRES<ChMatrixSPMV, Eigen::Lower | Eigen::Upper, Eigen::IdentityPreconditioner>* m_engine;
    ////Eigen::MINRES<ChMatrixSPMV, Eigen::Lower | Eigen::Upper, Eigen::DiagonalPreconditioner<double>>* m_engine;
};

}  // end namespace chrono

#endif
