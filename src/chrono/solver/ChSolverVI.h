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

#ifndef CH_SOLVER_VI_H
#define CH_SOLVER_VI_H

#include "chrono/solver/ChSolver.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// Base class for solvers aimed at solving complementarity problems arising from QP optimization problems.
class ChApi ChSolverVI : public ChSolver {
  public:
    virtual ~ChSolverVI() {}
};

/// @} chrono_solver

}  // end namespace chrono

#endif