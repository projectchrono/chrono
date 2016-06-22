// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSOLVERSIMPLEX_H
#define CHSOLVERSIMPLEX_H

#include "chrono/solver/ChSolver.h"

namespace chrono {

// forward reference
class ChLinkedListMatrix;
class ChUnilateralData;

///     ***OBSOLETE****
/// A simplex (pivoting) method which solves complementarity problems by activating/deactivating
/// constraints and solving a linear problem each time (using a custom sparse solver)
/// It is significantly slower than iterative methods, but it gives an almost exact solution
/// (within numerical roundoff and accumulation errors, of course). It can handle redundant constraints.
/// This solver must be used for mixed-linear complementarity problems (MLCP) in this form:
///
///    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
///    | Cq  0 | |l|  |-b|  |c|
///
///   as arising in the solution of QP with
///   inequalities or in multibody problems.

class ChApi ChSolverSimplex : public ChSolver {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChSolverSimplex, ChSolver);

  protected:
    int truncation_step;            ///< if 0 no effect, if > 0 steps are truncated
    ChLinkedListMatrix* MC;         ///< the sparse matrix for direct solution [MC]X=B (auto fill)
    ChMatrix<>* X;                  ///< the unknown vector (automatically filled)
    ChMatrix<>* B;                  ///< the known vector (automatically filled)
    ChUnilateralData* unilaterals;  ///< array with temporary info for pivoting

  public:
    ChSolverSimplex();
    ~ChSolverSimplex();

    /// Performs the solution of the problem, using the simplex method.
    /// If you must solve many problems with the same number of
    /// variables and constraints, we suggest you to use the same
    /// ChSolverSimplex object, because it exploits coherency: avoids
    /// reallocating the sparse matrix each time.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Set truncation step (that is, the method stops anyway after
    /// this amount of pivots in simplex explorations, even if the
    /// solution isn't reached). NOTE!! This limits cases with exponential
    /// complexity explosion, but premature termination can give _meaningless_
    /// results (differently from premature termination of iterative methods).
    /// For truncation step = 0 continue endlessly up to exact solution (default)
    void SetTruncationStep(int mstep) { truncation_step = mstep; }
    void SetNoTruncation() { truncation_step = 0; }
    double GetTruncationStep() { return truncation_step; }
};

}  // end namespace chrono

#endif
