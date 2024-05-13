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
// Base class for iterative solvers.
//
// =============================================================================

#ifndef CH_ITERATIVESOLVER_H
#define CH_ITERATIVESOLVER_H

#include "chrono/core/ChApiCE.h"
#include "chrono/solver/ChSystemDescriptor.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// Base class for iterative solvers.
/// Provides basic settings, common to both linear and complementarity iterative solvers.
class ChApi ChIterativeSolver {
  public:
    virtual ~ChIterativeSolver() {}

    /// Set the maximum number of iterations.
    virtual void SetMaxIterations(int max_iterations) { m_max_iterations = max_iterations; }

    /// Set the tolerance threshold used by the stopping criteria.
    void SetTolerance(double tolerance) { m_tolerance = tolerance; }

    /// Enable/disable use of a simple diagonal preconditioner (default: true).
    /// If enabled, solver that support this feature will use the diagonal of the system matrix for preconditioning.
    void EnableDiagonalPreconditioner(bool val) { m_use_precond = val; }

    /// Enable/disable warm starting by providing an initial guess (default: false).\n
    /// If enabled, the solvers use as an initial guess the current values for [x; -lambda].\n
    /// ATTENTION: enable this option **only** if using the Euler implicit linearized integrator!
    void EnableWarmStart(bool val) { m_warm_start = val; }

    /// Get the current maximum number of iterations.
    int GetMaxIterations() const { return m_max_iterations; }

    /// Get the current tolerance value.
    double GetTolerance() const { return m_tolerance; }

    /// Return the number of iterations performed during the last solve.
    virtual int GetIterations() const = 0;

    /// Return the tolerance error reached during the last solve.
    virtual double GetError() const = 0;

  protected:
    ChIterativeSolver(int max_iterations, double tolerance, bool use_precond, bool warm_start);

    // Debugging utilities
    void WriteMatrices(ChSystemDescriptor& sysd, bool one_indexed = true);
    double CheckSolution(ChSystemDescriptor& sysd, const ChVectorDynamic<>& x);

    bool m_use_precond;    ///< use diagonal preconditioning?
    bool m_warm_start;     ///< use initial guesss?
    int m_max_iterations;  ///< maximum number of iterations
    double m_tolerance;    ///< tolerance threshold in stopping criteria

    friend class ChSystem;
};

/// @} chrono_solver

}  // end namespace chrono

#endif
