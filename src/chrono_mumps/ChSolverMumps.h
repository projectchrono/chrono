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

#include "chrono_mumps/ChApiMumps.h"
#include "chrono_mumps/ChMumpsEngine.h"
#include "chrono/solver/ChDirectSolverLS.h"

namespace chrono {

/// @addtogroup mumps_module
/// @{


/** \class ChSolverMumps
\brief Interface to the MUMPS parallel sparse direct solver.

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
auto mumps_solver = chrono_types::make_shared<ChSolverMumps>();
system.SetSolver(mumps_solver);
\endcode

See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the solver.
*/
class ChApiMumps ChSolverMumps : public ChDirectSolverLS {
  public:
    ChSolverMumps(int num_threads = 1);
    ~ChSolverMumps() {}
    virtual Type GetType() const override { return Type::MUMPS; }

    /// Enable detection of null pivots.
    virtual void EnableNullPivotDetection(bool val, double threshold = 0) override;

    /// Set the matrix symmetry type (default: GENERAL).
    virtual void SetMatrixSymmetryType(MatrixSymmetryType symmetry) override;

    /// Get a handle to the underlying Mumps engine.
    ChMumpsEngine& GetMumpsEngine() { return m_engine; }

  private:
    /// Factorize the current sparse matrix and return true if successful.
    virtual bool FactorizeMatrix() override;

    /// Solve the linear system using the current factorization and right-hand side vector.
    /// Load the solution vector (already of appropriate size) and return true if succesful.
    virtual bool SolveSystem() override;

    /// Display an error message corresponding to the last failure.
    /// This function is only called if Factorize or Solve returned false.
    virtual void PrintErrorMessage() override;

    ChMumpsEngine m_engine;  ///< interface to Mumps solver
};

/// @} mumps_module

}  // namespace chrono

#endif
