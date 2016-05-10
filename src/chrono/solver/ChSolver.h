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

#ifndef CHSOLVER_H
#define CHSOLVER_H

#include <vector>
#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// Base class for solvers aimed at solving complementarity problems arising
/// from QP optimization problems.
/// This class does nothing: it is up to derived classes to implement specific
/// solution methods, such as iterative SOR, APGD, simplex, etc.
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
class ChApi ChSolver {
    // Chrono RTTI, needed for serialization
    CH_RTTI_ROOT(ChSolver);

  public:
    bool verbose;

    //
    // CONSTRUCTORS
    //

    ChSolver() { verbose = false; }

    virtual ~ChSolver() {}

    //
    // FUNCTIONS
    //

    // --Following functions are generic interfaces to the LCP solver. The
    //   Solve() function is a pure virtual method, so it MUST be implemented
    //   by specialized child classes:

    /// Performs the solution of the LCP.
    /// You must provide a system description using ChSystemDescriptor.
    /// This function MUST be implemented in children classes, with specialized
    /// methods such as iterative schemes, simplex schemes, fixed point algorithms, etc.
    /// \return  the maximum constraint violation after termination.

    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) = 0;

    /// This method is implemented in direct solvers such as MKL
    virtual double Factorize(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                             ) {
        return 0.0f;
    };

    //
    // Utility functions
    //

    void SetVerbose(bool mv) { this->verbose = mv; }
    bool GetVerbose() const { return this->verbose; }

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        // serialize all member data:
        marchive << CHNVP(verbose);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        // stream in all member data:
        marchive >> CHNVP(verbose);
    }
};

/// @} chrono_solver

}  // end namespace chrono

#endif