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
    //ChVectorDynamic<> L(nc);

    // penalty terms from constraints
    R.setZero(nv);
    //L.setZero(nc);
    
    sysdMD.BuildFbVector(R); // rhs, applied forces (plus the penalty terms of constraints if timestepper in penalty ON mode)

    ChVectorDynamic<> diag_all;
    sysdMD.BuildDiagonalVector(diag_all);
    diagonal_M = diag_all.head(nv);

    constexpr bool apply_partition_weights = false;
    if (apply_partition_weights) {
        if (auto domain = sysdMD.GetDomain()) {
            domain->ComputeSharedCoordsWeights(domain->CoordWeightsWv());
            const auto& wv = domain->CoordWeightsWv();
            if (wv.size() >= nv) {
                for (int i = 0; i < nv; ++i) {
                    const double w = wv(i);
                    R[i] *= w;
                    diagonal_M[i] *= w;
                }
            }
        }
    }

    // MULTIDOMAIN: assemble shared force and lumped mass prior to division.
    // This avoids re-adding already divided accelerations across interfaces.
    sysdMD.VectAdditiveToClipped(R);
    sysdMD.VectAdditiveToClipped(this->diagonal_M);

    // Compute acceleration using lumped diagonal mass. No need to invoke a linear solver.
    //   a = Md^-1 * F
    // It can be done compactly via   a.array() = R.array() / this->diagonal_M.array();   but we rather do:

    for (int i = 0; i < R.size(); ++i) {
        a[i] = R[i] / this->diagonal_M[i];

        // Hack to overcome a 0/0=NaN result for FEA like IGA where one node is a domain outlier (hence zero R) but with zero mass M because the element that gives the mass is in the other domain
        if (this->diagonal_M[i] == 0)
            a[i] = 0; 
    }

    // Synchronize shared accelerations via interface averaging.
    sysdMD.VectAdditiveToClipped(a, 1.0);

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
