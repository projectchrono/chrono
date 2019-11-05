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

#include "chrono_mkl/ChApiMkl.h"
#include "chrono/solver/ChSolverDirect.h"

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
class ChApiMkl ChSolverMKL : public ChSolverDirect {
  public:
    ChSolverMKL() {}
    ~ChSolverMKL() {}

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

/// @} mkl_module

}  // end namespace chrono

#endif
