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

#ifndef CHSOLVERMKL_H
#define CHSOLVERMKL_H

#include "chrono_pardisomkl/ChApiPardisoMKL.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChDirectSolverLScomplex.h"

#include <Eigen/PardisoSupport>

namespace chrono {

/// @addtogroup pardisomkl_module
/// @{

/** \class ChSolverPardisoMKL
\brief Interface to the Intel MKL Pardiso parallel sparse direct solver.

Sparse linear direct solver.
Cannot handle VI and complementarity problems, so it cannot be used with NSC formulations.

The solver is equipped with two main features:
- sparsity pattern lock
- sparsity pattern learning

See ChDirectSolverLS for more details.

<div class="ce-warning">
If appropriate and warranted by the problem setup, it is \e highly recommended to enable the sparsity pattern \e lock.
This can significantly improve performance for more complex problems (larger size and/or problems which include
constraints).
</div>

Minimal usage example, to be put anywhere in the code, before starting the main simulation loop:
\code{.cpp}
auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
system.SetSolver(mkl_solver);
\endcode

See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the solver.
*/
class ChApiPardisoMKL ChSolverPardisoMKL : public ChDirectSolverLS {
  public:
    /// Construct an MKL Pardiso sparse direct solver object and specify the number of OpenMP threads.
    /// Passing the default value num_threads=0 results in using a number of threads equal to the number
    /// of available processors (as returned by the function omp_get_num_procs)
    ChSolverPardisoMKL(int num_threads = 0);

    ~ChSolverPardisoMKL() {}

    virtual Type GetType() const override { return Type::PARDISO_MKL; }

    /// Get a handle to the underlying MKL engine.
    Eigen::PardisoLU<ChSparseMatrix>& GetMklEngine() { return m_engine; }

  private:
    /// Factorize the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() override;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem() override;

    /// Display an error message corresponding to the last failure.
    /// This function is only called if Factorize or Solve returned false.
    virtual void PrintErrorMessage() override;

    Eigen::PardisoLU<ChSparseMatrix> m_engine;  ///< underlying Eigen Pardiso interface
};



/// Sparse complex Pardiso direct solver.\n
class ChApiPardisoMKL ChSolverComplexPardisoMKL : public ChDirectSolverLScomplex {
  public:
      ChSolverComplexPardisoMKL(int num_threads = 0);
    ~ChSolverComplexPardisoMKL() {}

    /// Get a handle to the underlying MKL engine.
    Eigen::PardisoLU<Eigen::SparseMatrix<std::complex<double>, Eigen::ColMajor>>& GetMklEngine() { return m_engine; }

  private:
    /// Factorize the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() override;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem(const ChVectorDynamic<std::complex<double>>& b) override;

    /// Display an error message corresponding to the last failure.
    /// This function is only called if Factorize or Solve returned false.
    virtual void PrintErrorMessage() override;

    Eigen::PardisoLU<Eigen::SparseMatrix<std::complex<double>, Eigen::ColMajor>> m_engine;  ///< underlying Eigen Pardiso interface
};


/// @} mkl_module

}  // end namespace chrono

#endif
