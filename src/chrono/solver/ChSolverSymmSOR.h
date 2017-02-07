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

#ifndef CHSOLVERSYMMSOR_H
#define CHSOLVERSYMMSOR_H

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// An iterative solver based on symmetric projective fixed point method, with overrelaxation
/// and immediate variable update as in SSOR methods.
/// The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y
///  | Cq -E | |l|  |-b|  |c|
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones

class ChApi ChSolverSymmSOR : public ChIterativeSolver {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSolverSymmSOR)

  public:
    ChSolverSymmSOR(int mmax_iters = 50,       ///< max.number of iterations
                    bool mwarm_start = false,  ///< uses warm start?
                    double mtolerance = 0.0,   ///< tolerance for termination criterion
                    double momega = 1.0        ///< overrelaxation criterion
                    )
        : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, momega) {}

    virtual ~ChSolverSymmSOR() {}

    virtual Type GetType() const override { return Type::SYMMSOR; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;
};

}  // end namespace chrono

#endif
