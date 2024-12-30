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

#include "chrono_multidomain/ChSolverLumpedMultidomain.h"
#include "chrono_multidomain/ChSystemDescriptorMultidomain.h"
#include "chrono/serialization/ChArchiveBinary.h"

namespace chrono {
namespace multidomain {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverLumpedMultidomain)
CH_UPCASTING(ChSolverLumpedMultidomain, ChSolver)

ChSolverLumpedMultidomain::ChSolverLumpedMultidomain() {}

double ChSolverLumpedMultidomain::Solve(ChSystemDescriptor& sysd) {

    ChSystemDescriptorMultidomain& sysdMD = dynamic_cast<ChSystemDescriptorMultidomain&>(sysd);

    // Allocate auxiliary vectors;

    int nc = sysd.CountActiveConstraints();
    int nv = sysd.CountActiveVariables();

    if (verbose)
        std::cout << "\n-----ChSolverLumpedMultidomain, nv=" << nv << " vars, nc=" << nc << " penalty constraints." << std::endl;

    ChVectorDynamic<> R(nv);
    ChVectorDynamic<> a(nv);
    //ChVectorDynamic<> L(nc);

    // penalty terms from constraints
    R.setZero(nv);
    //L.setZero(nc);
    
    sysdMD.BuildFbVector(R); // rhs, applied forces (plus the penalty terms of constraints if timestepper in penalty ON mode)

    // Compute acceleration using lumped diagonal mass. No need to invoke a linear solver.
    //   a = Md^-1 * F
    // It can be done compactly via   a.array() = R.array() / this->diagonal_M.array();   but we rather do:

    for (int i = 0; i < R.size(); ++i) {
        a[i] = R[i] / this->diagonal_M[i];

        // Hack to overcome a 0/0=NaN result for FEA like IGA where one node is a domain outlier (hence zero R) but with zero mass M because the element that gives the mass is in the other domain
        if (this->diagonal_M[i] == 0)
            a[i] = 0; 
    }


    // MULTIDOMAIN******************
    // Sum the contribution to acceleration from other domains. Alternatively we could have summed R. 
    // That's all. Lucky situation because we assume that the Md array is in distributed, not additive format.
    sysdMD.VectAdditiveToDistributed(a);  

    // Resulting dual variables:
    //sysd.FromVectorToConstraints(L);

    // Resulting primal variables:
    sysd.FromVectorToVariables(a);

    if (verbose)
        std::cout << "-----" << std::endl;

    return 0;
}



}  // end namespace multidomain
}  // end namespace chrono
