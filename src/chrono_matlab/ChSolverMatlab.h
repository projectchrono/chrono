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

#ifndef CHSOLVERMATLAB_H
#define CHSOLVERMATLAB_H

#include "chrono_matlab/ChApiMatlab.h"
#include "chrono_matlab/ChMatlabEngine.h"

#include "chrono/solver/ChSolver.h"

namespace chrono {

/// @addtogroup matlab_module
/// @{

/// Class for using a Matlab linear solver from Chrono programs.
class ChApiMatlab ChSolverMatlab : public ChSolver {

  protected:
    ChMatlabEngine* mengine;

  public:
    ChSolverMatlab(ChMatlabEngine& me);
    ChSolverMatlab();

    virtual ~ChSolverMatlab() {}

    /// Set the Matlab engine.
    void SetEngine(ChMatlabEngine* me) { mengine = me; }

    /// Indicate whether or not the Solve() phase requires an up-to-date problem matrix.
    /// As typical of direct solvers, the Pardiso solver only requires the matrix for its Setup() phase.
    virtual bool SolveRequiresMatrix() const override { return false; }

    /// Solve using the Matlab default direct solver (as in x=A\\b)
    virtual double Solve(ChSystemDescriptor& sysd) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

/// @} matlab_module

}  // end namespace chrono

#endif
