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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CH_DIRECTSOLVER_LS_COMPLEX_H
#define CH_DIRECTSOLVER_LS_COMPLEX_H

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/solver/ChSolverLS.h"

#include <Eigen/SparseLU>

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/** \class ChDirectSolverLScomplex
\brief Base class for sparse direct linear solvers with complex coefficients.

Sparse linear direct solvers A*x=b for complex A,b,x.

These solvers are mostly meant to work in problems of finding eigenpairs, where
one might need to compute the shift-and-invert operator during Arnoldi or Krylov-Schur
iterations, that might have complex shifts.
*/

class ChApi ChDirectSolverLScomplex  {
  public:
    
    virtual ~ChDirectSolverLScomplex() {}

    /// Get shortcut handle to underlying A matrix, for A*x=b
    Eigen::SparseMatrix<std::complex<double>, Eigen::ColMajor>& A() { return m_mat; }

    /// Get shortcut handle to underlying x solution vector, for A*x=b
    ChVectorDynamic<std::complex<double>>& x() { return m_sol; }

    /// Perform the solver setup operations (factorization)
    /// Returns true if successful and false otherwise.
    virtual bool Setup();

    /// Solve linear system.
    virtual double Solve(const ChVectorDynamic<std::complex<double>>& b);

    bool verbose = false;

  protected:
    ChDirectSolverLScomplex();

    /// Factorize the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() = 0;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem(const ChVectorDynamic<std::complex<double>>& b) = 0;

    /// Display an error message corresponding to the last failure.
    /// This function is only called if Factorize or Solve returned false.
    virtual void PrintErrorMessage() = 0;

   
    Eigen::SparseMatrix<std::complex<double>, Eigen::ColMajor> m_mat;           ///< problem matrix
    int m_dim;                      ///< problem size
    ChVectorDynamic<std::complex<double>> m_sol;  ///< solution vector
};

// ---------------------------------------------------------------------------

/// Sparse complex LU direct solver.\n
/// Interface to Eigen's SparseLU solver, a supernodal LU factorization for general matrices.\n
/// Cannot handle VI and complementarity problems, so it cannot be used with NSC formulations.\n
/// See ChDirectSolverLS for more details.
class ChApi ChSolverSparseComplexLU : public ChDirectSolverLScomplex {
  public:
    ChSolverSparseComplexLU() {}
    ~ChSolverSparseComplexLU() {}

  private:
    /// Factorize the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() override;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem(const ChVectorDynamic<std::complex<double>>& b) override;

    /// Display an error message corresponding to the last failure.
    /// This function is only called if Factorize or Solve returned false.
    virtual void PrintErrorMessage() override;

    Eigen::SparseLU<Eigen::SparseMatrix<std::complex<double>, Eigen::ColMajor>, Eigen::COLAMDOrdering<int>> m_engine;  ///< Eigen SparseLU solver
};

/// Sparse complex QR direct solver.\n
/// Interface to Eigen's SparseQR solver, a left-looking rank-revealing QR factorization.\n
/// Cannot handle VI and complementarity problems, so it cannot be used with NSC formulations.\n
/// See ChDirectSolverLS for more details.
class ChApi ChSolverSparseComplexQR : public ChDirectSolverLScomplex {
  public:
    ChSolverSparseComplexQR() {}
    ~ChSolverSparseComplexQR() {}

  private:
    /// Factorize the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() override;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem(const ChVectorDynamic<std::complex<double>>& b) override;

    /// Display an error message corresponding to the last failure.
    /// This function is only called if Factorize or Solve returned false.
    virtual void PrintErrorMessage() override;

    Eigen::SparseQR<Eigen::SparseMatrix<std::complex<double>, Eigen::ColMajor>, Eigen::COLAMDOrdering<int>> m_engine;  ///< Eigen SparseQR solver
};

/// @} chrono_solver

}  // end namespace chrono

#endif
