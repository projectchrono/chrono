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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/solver/ChSolverLumped.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverLumped)
CH_UPCASTING(ChSolverLumped, ChSolver)

ChSolverLumped::ChSolverLumped() {}

double ChSolverLumped::Solve(ChSystemDescriptor& sysd) {

    // Allocate auxiliary vectors;

    int nc = sysd.CountActiveConstraints();
    int nv = sysd.CountActiveVariables();

    if (verbose)
        std::cout << "\n-----ChSolverLumped, nv=" << nv << " vars, nc=" << nc << " penalty constraints." << std::endl;


    ChVectorDynamic<> R(nv);
    ChVectorDynamic<> a(nv);

    // penalty terms from constraints
    R.setZero(nv);
    
    sysd.BuildFbVector(R); // rhs, applied forces

    // compute acceleration using lumped diagonal mass. No need to invoke a linear solver.
    a.array() = R.array() / this->diagonal_M.array();  //  a = Md^-1 * F


    // Resulting primal variables:
    sysd.FromVectorToVariables(a);

    if (verbose)
        std::cout << "-----" << std::endl;

    return 0;
}


}  // end namespace chrono
