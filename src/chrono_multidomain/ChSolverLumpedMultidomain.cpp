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
#include "chrono_multidomain/ChDomain.h"
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

    // penalty terms from constraints
    R.setZero(nv);
    
    sysdMD.BuildFbVector(R); // rhs, applied forces (maybe plus the constraint penalty if timestepper in penalty ON mode)

    // NO NOT NEEDED... (old code by Matthew) When ChSystem::StateSolveCorrection() is called in outer context, it detects that 
    // it already takes this->diagonal_M and it fills it with the lumped masses of the system. 
    // Also note that the lumped masses are already split between shared domains with Wv weights.
    /*
    ChVectorDynamic<> diag_all;
    sysdMD.BuildDiagonalVector(diag_all);  // POTENTIAL TROUBLE! THIS IS NOT NECESSARILY THE SAME AS DIAGONAL MASSES: MORE IN GENERAL
                                           // IT GIVES THE DIAGONAL OF THE "H" MATRIX, THAT IS OFTEN M, BUT SOMETIMES ALSO WITH K AND R 
    diagonal_M = diag_all.head(nv);
    ...
    */

    // MULTIDOMAIN_COMM************
    // Assembly shared forces, so that the following a = R / diagonal_M is correct
    sysdMD.VectAdditiveToClipped(R);

    // MULTIDOMAIN_COMM************
    // Assembly shared masses, so that the following a = R / diagonal_M is correct
    // Note: such diagonal_M is in "additive" form because diagonal_M was populated by 
    // ChSystem::StateSolveCorrection() in the outer context where it calls ChSystem::LoadLumpedMass_Md() 
    // and assuming one has set system->EnableCoordWeightsWv(true), then LoadLumpedMass_Md does this:
    //  - the masses of bodies and nodes (topology vertexes, shared) are scaled by the Wv weights
    //  - the masses introduced by FEA elements (topology edges) are NOT scaled by the Wv weights, that
    //    is good because they are not shared and live in just one of the domains.
    // So we only need to convert the diagonal_M from additive to clipped form.
    sysdMD.VectAdditiveToClipped(diagonal_M);  


    // Compute acceleration using lumped diagonal mass. No need to invoke a linear solver.
    //   a = Md^-1 * F
    // For diagonal Md, this is equivalent to
    //   a_clipped = (Md_clipped)^-1 * F_clipped
    // It can be done compactly via   a.array() = R.array() / this->diagonal_M.array();   but we rather do:

    for (int i = 0; i < R.size(); ++i) {
        a[i] = R[i] / this->diagonal_M[i];

        // Hack to overcome a 0/0=NaN result for FEA like IGA where one node is a domain outlier (hence zero R) but with zero mass M because the element that gives the mass is in the other domain
        // NOTE: THIS SHOULD NOT BE NEEDED ANYMORE BECAUSE WE WORK WITH diagonal_M IN "CLIPPED" FORM AFTER VectAdditiveToClipped 
        //if (this->diagonal_M[i] == 0)
        //    a[i] = 0; 
    }

    // MULTIDOMAIN_COMM************
    // Synchronize shared accelerations via interface averaging. In theory the shared accelerations should be equal across nodes, but
    // in practice they may differ due to numerical errors, e.g. finite precision in fp arithmetic, different processors, etc., so we average them across domains.
    sysdMD.VectAverage(a, 1.0);

    // Resulting primal variables:
    sysd.FromVectorToVariables(a);

    if (verbose)
        std::cout << "-----" << std::endl;

    return 0;
}



}  // end namespace multidomain
}  // end namespace chrono
