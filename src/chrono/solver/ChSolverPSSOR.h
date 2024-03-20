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

#ifndef CHSOLVERSYMMSOR_H
#define CHSOLVERSYMMSOR_H

#include "chrono/solver/ChIterativeSolverVI.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// An iterative solver based on symmetric projective fixed point method, with overrelaxation and immediate variable
/// update as in SSOR methods.\n
/// See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the
/// solver.
/// \deprecated Use ChSolverPSOR, ChSolverBB, or ChSolverAPGD instead. This class will be removed in a future Chrono
/// release.
class ChApi
/// \cond
CH_DEPRECATED("deprecated. Use ChSolverPSOR, ChSolverBB, or ChSolverAPGD instead.")
    /// \endcond
    ChSolverPSSOR : public ChIterativeSolverVI {
  public:
    ChSolverPSSOR();

    ~ChSolverPSSOR() {}

    virtual Type GetType() const override { return Type::PSSOR; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Return the tolerance error reached during the last solve.
    /// For the PSSOR solver, this is the maximum constraint violation.
    virtual double GetError() const override { return maxviolation; }

  private:
    double maxviolation;
};

/// @} chrono_solver

}  // end namespace chrono

#endif
