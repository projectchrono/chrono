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

#include "chrono/physics/ChLoaderUV.h"

namespace chrono {

void ChLoaderUVdistributed::ComputeQ(ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    Q.setZero(loadable->GetLoadableNumCoordsVelLevel());
    ChVectorDynamic<> mF(loadable->GetNumFieldCoords());
    mF.setZero();

    if (!loadable->IsTriangleIntegrationNeeded()) {
        // Case of normal quadrilateral isoparametric coords
        assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTables()->Weight.size());
        assert(GetIntegrationPointsV() <= ChQuadrature::GetStaticTables()->Weight.size());
        const std::vector<double>& Ulroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsU() - 1];
        const std::vector<double>& Uweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsU() - 1];
        const std::vector<double>& Vlroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsV() - 1];
        const std::vector<double>& Vweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsV() - 1];

        ChVectorDynamic<> mNF(Q.size());  // temporary value for loop

        // Gauss quadrature :  Q = sum (N'*F*detJ * wi*wj)
        for (unsigned int iu = 0; iu < Ulroots.size(); iu++) {
            for (unsigned int iv = 0; iv < Vlroots.size(); iv++) {
                double detJ;
                // Compute F= F(u,v)
                ComputeF(Ulroots[iu], Vlroots[iv], mF, state_x, state_w);
                // Compute mNF= N(u,v)'*F
                loadable->ComputeNF(Ulroots[iu], Vlroots[iv], mNF, detJ, mF, state_x, state_w);
                // Compute Q+= mNF detJ * wi*wj
                mNF *= (detJ * Uweight[iu] * Vweight[iv]);
                Q += mNF;
            }
        }
    } else {
        // case of triangle: use special 3d quadrature tables (given U,V,W orders, use the U only)
        assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTablesTriangle()->Weight.size());
        const std::vector<double>& Ulroots =
            ChQuadrature::GetStaticTablesTriangle()->LrootsU[GetIntegrationPointsU() - 1];
        const std::vector<double>& Vlroots =
            ChQuadrature::GetStaticTablesTriangle()->LrootsV[GetIntegrationPointsU() - 1];
        const std::vector<double>& weight =
            ChQuadrature::GetStaticTablesTriangle()->Weight[GetIntegrationPointsU() - 1];

        ChVectorDynamic<> mNF(Q.size());  // temporary value for loop

        // Gauss quadrature :  Q = sum (N'*F*detJ * wi *1/2)   often detJ= 2 * triangle area
        for (unsigned int i = 0; i < Ulroots.size(); i++) {
            double detJ;
            // Compute F= F(u,v)
            ComputeF(Ulroots[i], Vlroots[i], mF, state_x, state_w);
            // Compute mNF= N(u,v)'*F
            loadable->ComputeNF(Ulroots[i], Vlroots[i], mNF, detJ, mF, state_x, state_w);
            // Compute Q+= mNF detJ * wi *1/2
            mNF *= (detJ * weight[i] * (1. / 2.));  // (the 1/2 coefficient is not in the table);
            Q += mNF;
        }
    }
}

void ChLoaderUVatomic::ComputeQ(ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    // Compute F=F(u,v)
    ChVectorDynamic<> F(loadable->GetNumFieldCoords());
    F.setZero();
    ComputeF(Pu, Pv, F, state_x, state_w);

    // Compute N(u,v)'*F
    double detJ;
    Q.setZero(loadable->GetLoadableNumCoordsVelLevel());
    loadable->ComputeNF(Pu, Pv, Q, detJ, F, state_x, state_w);
}

void ChLoaderUVatomic::SetApplication(double u, double v) {
    Pu = u;
    Pv = v;
}

}  // end namespace chrono
