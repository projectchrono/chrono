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

#include "chrono/physics/ChLoaderU.h"

namespace chrono {

void ChLoaderUdistributed::ComputeQ(ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTables()->Lroots.size());

    Q.setZero(loadable->GetLoadableNumCoordsVelLevel());
    ChVectorDynamic<> mF(loadable->GetNumFieldCoords());
    mF.setZero();

    const std::vector<double>& Ulroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsU() - 1];
    const std::vector<double>& Uweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsU() - 1];

    ChVectorDynamic<> mNF(Q.size());  // temporary value for loop

    // Gauss quadrature :  Q = sum (N'*F*detJ * wi)
    for (unsigned int iu = 0; iu < Ulroots.size(); iu++) {
        double detJ;
        // Compute F= F(u)
        ComputeF(Ulroots[iu], mF, state_x, state_w);
        // Compute mNF= N(u)'*F
        loadable->ComputeNF(Ulroots[iu], mNF, detJ, mF, state_x, state_w);
        // Compute Q+= mNF detJ * wi
        mNF *= (detJ * Uweight[iu]);
        Q += mNF;
    }
}

void ChLoaderUatomic::ComputeQ(ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    // Compute F=F(u)
    ChVectorDynamic<> F(loadable->GetNumFieldCoords());
    F.setZero();
    ComputeF(Pu, F, state_x, state_w);

    // Compute N(u)'*F
    double detJ;
    Q.setZero(loadable->GetLoadableNumCoordsVelLevel());
    loadable->ComputeNF(Pu, Q, detJ, F, state_x, state_w);
}

}  // end namespace chrono
