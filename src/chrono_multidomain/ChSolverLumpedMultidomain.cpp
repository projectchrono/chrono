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

    ChVectorDynamic<> Md(nv);  // diagonal mass
    Md.setZero(nv);

    ChVectorDynamic<> R(nv);
    ChVectorDynamic<> a(nv);
    ChVectorDynamic<> L(nc);

    // penalty terms from constraints
    R.setZero(nv);
    L.setZero(nc);
    //sysdMD.BuildBiVector(R); // rhs, applied forces, or...
    sysdMD.GetDomain()->GetSystem()->LoadResidual_F(R, 1.0);  // rhs, applied forces

    double err = 0;
    sysdMD.GetDomain()->GetSystem()->LoadLumpedMass_Md(Md, err, 1.0);


    if (nc) {
        sysdMD.GetDomain()->GetSystem()->LoadConstraint_C(L, -this->C_penalty_k);  // L  = -k*C     // to do: modulate  k  as constraint-dependent,
        // k=lumping->Ck_penalty*Ck_i
        sysdMD.GetDomain()->GetSystem()->LoadResidual_CqL(R, L, 1.0);                // Fc =  Cq' * (-k*C)    = Cq' * L
        // to do: add c_penalty for speed proportional damping, as   Fc = - Cq' * (k*C + c*(dC/dt))
    }

    // compute acceleration using lumped diagonal mass. No need to invoke a linear solver.
    a.array() = R.array() / Md.array();  //  a = Md^-1 * F

    // MULTIDOMAIN******************
    // Sum the contribution to acceleration from other domains. Alternatively we could have summed R. 
    // That's all. Lucky situation because we assume that the Md array is in distributed, not additive format.
    sysdMD.VectAdditiveToDistributed(a);  

    // Resulting dual variables:
    sysd.FromVectorToConstraints(L);

    // Resulting primal variables:
    sysd.FromVectorToVariables(a);

    if (verbose)
        std::cout << "-----" << std::endl;

    return 0;
}



}  // end namespace multidomain
}  // end namespace chrono
