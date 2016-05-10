//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSOLVERSOR_H
#define CHSOLVERSOR_H

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// An iterative solver based on projective fixed point method, with overrelaxation
/// and immediate variable update as in SOR methods.
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

class ChApi ChSolverSOR : public ChIterativeSolver {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChSolverSOR, ChIterativeSolver);

  public:
    //
    // CONSTRUCTORS
    //

    ChSolverSOR(int mmax_iters = 50,       ///< max.number of iterations
                bool mwarm_start = false,  ///< uses warm start?
                double mtolerance = 0.0,   ///< tolerance for termination criterion
                double momega = 1.0        ///< overrelaxation criterion
                )
        : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, momega) {}

    virtual ~ChSolverSOR() {}

    //
    // FUNCTIONS
    //

    /// Performs the solution of the LCP.
    /// \return  the maximum constraint violation after termination.

    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         );
};

}  // end namespace chrono

#endif
