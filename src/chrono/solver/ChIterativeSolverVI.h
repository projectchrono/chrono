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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_ITERATIVESOLVER_VI_H
#define CH_ITERATIVESOLVER_VI_H

#include "chrono/solver/ChSolverVI.h"
#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{
    
/** \class ChIterativeSolverVI
\brief Base class for iterative solvers aimed at solving complementarity problems arising from QP optimization problems.

The problem is described by a variational inequality VI(Z*x-d,K):\n

<pre>
  | M -Cq'|*|q|- | f|= |0|
  | Cq -E | |l|  |-b|  |c|
</pre>
with l \f$\in\f$ Y, C \f$\in\f$ Ny, normal cone to Y\n

Also Z symmetric by flipping sign of l_i:
<pre>
  |M  Cq'|*| q|-| f|=|0|
  |Cq  E | |-l| |-b| |c|
</pre>

- case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
- case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
- case CCP: Y_i are friction cones

The default maximum number of iterations is 50.

The 'tolerance' threshold value used in the stopping criteria may have different meaning for different solvers
(threshold on maximum constraint violation, threshold on projected residual, etc). See the GetError method for an
individual iterative VI solver for details.

Diagonal preconditioning is enabled by default, but may not supported by all iterative VI solvers.
*/
class ChApi ChIterativeSolverVI : public ChIterativeSolver, public ChSolverVI {
  public:
    ChIterativeSolverVI();

    virtual ~ChIterativeSolverVI() {}

    /// Set the overrelaxation factor (default: 1.0).
    /// This factor may be used by PSOR-like methods. A good value for Jacobi solver is 0.2; for other iterative solvers
    /// it can be up to 1.0
    void SetOmega(double mval);

    /// Set the sharpness factor (default: 1.0).
    /// This factor may be used by PSOR-like methods with projection (see Mangasarian LCP method). A good sharpness
    /// value is in the 0.8 ... 1.0 range (lower values improve accuracy but at the cost of slower convergence)
    void SetSharpnessLambda(double mval);

    /// Enable/disable recording of the constraint violation history.
    /// If enabled, the maximum constraint violation at the end of each iteration is stored in a vector (see
    /// GetViolationHistory).
    void SetRecordViolation(bool mval) { record_violation_history = mval; }

    /// Return the current value of the overrelaxation factor.
    double GetOmega() const { return m_omega; }

    /// Return the current value of the sharpness factor.
    double GetSharpnessLambda() const { return m_shlambda; }

    /// Return the number of iterations performed during the last solve.
    virtual int GetIterations() const override { return m_iterations; }

    /// Access the vector of constraint violation history.
    /// Note that collection of constraint violations must be enabled through SetRecordViolation.
    const std::vector<double>& GetViolationHistory() const { return violation_history; }

    /// Access the vector with history of maximum change in Lagrange multipliers
    /// Note that collection of constraint violations must be enabled through SetRecordViolation.
    const std::vector<double>& GetDeltalambdaHistory() const { return dlambda_history; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  protected:
    /// This method MUST be called by all iterative methods INSIDE their iteration loops
    /// (at the end). If history recording is enabled, this function will store the
    /// current values as passed as arguments.
    /// Note: 'iternum' starts at 0 for the first iteration.
    void AtIterationEnd(double mmaxviolation, double mdeltalambda, unsigned int iternum);

  protected:
    /// Indicate whether ot not the Solve() phase requires an up-to-date problem matrix.
    /// Typically, this is the case for iterative solvers (as the matrix is needed for
    /// the matrix-vector operations).
    virtual bool SolveRequiresMatrix() const override { return true; }

    int m_iterations;   ///< total number of iterations performed by the solver
    double m_omega;     ///< over-relaxation factor
    double m_shlambda;  ///< sharpness factor

    bool record_violation_history;
    std::vector<double> violation_history;
    std::vector<double> dlambda_history;
};

/// @} chrono_solver

}  // end namespace chrono

#endif
