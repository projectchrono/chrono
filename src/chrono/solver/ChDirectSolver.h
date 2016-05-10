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

#ifndef CHLCPDIRECTSOLVER_H
#define CHLCPDIRECTSOLVER_H

#include "chrono/solver/ChSolver.h"

namespace chrono {

///    Base class for DIRECT solvers aimed at solving
///   LCP linear complementarity problems arising
///   from QP optimization problems.
///    This class does nothing: it is up to inherited
///   classes to implement specific solution methods,
///   such as simplex, iterative SOR, etc.
///    The LCP problem must be in this (symmetric)
///   form:
///
///    | M -Cq'|*|q|- | f|= |0| ,   c>=0, l>=0, l*c=0;
///    | Cq  0 | |l|  |-b|  |c|
///
///   as arising in the solution of QP with
///   inequalities or in multibody problems.

class ChApi ChLcpDirectSolver : public ChLcpSolver {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChLcpDirectSolver, ChLcpSolver);

  protected:
    //
    // DATA
    //

  public:
    //
    // CONSTRUCTORS
    //

    ChLcpDirectSolver(){};

    virtual ~ChLcpDirectSolver(){};

    //
    // FUNCTIONS
    //
};

}  // end namespace chrono

#endif