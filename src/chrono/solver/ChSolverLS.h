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

#ifndef CH_SOLVER_LS_H
#define CH_SOLVER_LS_H

#include "chrono/solver/ChSolver.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// Base class for solvers aimed at solving linear systems.
class ChApi ChSolverLS : public ChSolver {
  public:
    virtual ~ChSolverLS() {}
};

/// @} chrono_solver

}  // end namespace chrono

#endif