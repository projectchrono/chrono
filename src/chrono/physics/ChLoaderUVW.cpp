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

#include "chrono/physics/ChLoaderUVW.h"

namespace chrono {

void ChLoaderUVWdistributed::ComputeQ(ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    Q.setZero(loadable->GetLoadableNumCoordsVelLevel());
    ChVectorDynamic<> mF(loadable->GetNumFieldCoords());
    mF.setZero();

    if (loadable->IsTetrahedronIntegrationNeeded()) {
        // case of tetrahedron: use special 3d quadrature tables (given U,V,W orders, use the U only)
        assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTablesTetrahedron()->Weight.size());
        const std::vector<double>& Ulroots =
            ChQuadrature::GetStaticTablesTetrahedron()->LrootsU[GetIntegrationPointsU() - 1];
        const std::vector<double>& Vlroots =
            ChQuadrature::GetStaticTablesTetrahedron()->LrootsV[GetIntegrationPointsU() - 1];
        const std::vector<double>& Wlroots =
            ChQuadrature::GetStaticTablesTetrahedron()->LrootsW[GetIntegrationPointsU() - 1];
        const std::vector<double>& weight =
            ChQuadrature::GetStaticTablesTetrahedron()->Weight[GetIntegrationPointsU() - 1];

        ChVectorDynamic<> mNF(Q.size());  // temporary value for loop

        // Gauss quadrature :  Q = sum (N'*F*detJ * wi * 1/6)   often detJ=6*tetrahedron volume
        for (unsigned int i = 0; i < Ulroots.size(); i++) {
            double detJ;
            // Compute F= F(u,v,w)
            ComputeF(Ulroots[i], Vlroots[i], Wlroots[i], mF, state_x, state_w);
            // Compute mNF= N(u,v,w)'*F
            loadable->ComputeNF(Ulroots[i], Vlroots[i], Wlroots[i], mNF, detJ, mF, state_x, state_w);
            // Compute Q+= mNF * detJ * wi * 1/6
            mNF *= (detJ * weight[i] * CH_1_6);  // (the 1/6 coefficient is not in the table);
            Q += mNF;
        }
    } else if (loadable->IsTrianglePrismIntegrationNeeded()) {
        // Case of triangle prism: quadrature on u,v in [0..1] for the triangle, on w in [-1...+1] for thickness.
        assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTablesTriangle()->Weight.size());
        const std::vector<double>& Ulroots =
            ChQuadrature::GetStaticTablesTriangle()->LrootsU[GetIntegrationPointsU() - 1];
        const std::vector<double>& Vlroots =
            ChQuadrature::GetStaticTablesTriangle()->LrootsV[GetIntegrationPointsU() - 1];
        const std::vector<double>& weight =
            ChQuadrature::GetStaticTablesTriangle()->Weight[GetIntegrationPointsU() - 1];
        assert(GetIntegrationPointsW() <= ChQuadrature::GetStaticTables()->Lroots.size());
        const std::vector<double>& Wlroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsW() - 1];
        const std::vector<double>& Wweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsW() - 1];

        ChVectorDynamic<> mNF(Q.size());  // temporary value for loop

        // Gauss quadrature :  Q = sum (N'*F*detJ * wi * wj *1/2)   often detJ= 2 * triangle area
        // This requires a single outer for loop over all triangle points, already queued in arrays Ulroots Vlroots,
        // and an inner loop over thickness points Wlroots.
        for (unsigned int i = 0; i < Ulroots.size(); i++) {
            for (unsigned int iw = 0; iw < Wlroots.size(); iw++) {
                double detJ;
                // Compute F= F(u,v)
                ComputeF(Ulroots[i], Vlroots[i], Vlroots[iw], mF, state_x, state_w);
                // Compute mNF= N(u,v)'*F
                loadable->ComputeNF(Ulroots[i], Vlroots[i], Vlroots[iw], mNF, detJ, mF, state_x, state_w);
                // Compute Q+= mNF * detJ * wi *1/2
                mNF *= (detJ * weight[i] * Wweight[iw] *
                        (1. / 2.));  // (the 1/2 coefficient is not in the triangle table);
                Q += mNF;
            }
        }
    } else {
        // Case of normal box isoparametric coords (default)
        assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTables()->Lroots.size());
        assert(GetIntegrationPointsV() <= ChQuadrature::GetStaticTables()->Lroots.size());
        assert(GetIntegrationPointsW() <= ChQuadrature::GetStaticTables()->Lroots.size());
        const std::vector<double>& Ulroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsU() - 1];
        const std::vector<double>& Uweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsU() - 1];
        const std::vector<double>& Vlroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsV() - 1];
        const std::vector<double>& Vweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsV() - 1];
        const std::vector<double>& Wlroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsW() - 1];
        const std::vector<double>& Wweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsW() - 1];

        ChVectorDynamic<> mNF(Q.size());  // temporary value for loop

        // Gauss quadrature :  Q = sum (N'*F*detJ * wi*wj*wk)
        for (unsigned int iu = 0; iu < Ulroots.size(); iu++) {
            for (unsigned int iv = 0; iv < Vlroots.size(); iv++) {
                for (unsigned int iw = 0; iw < Wlroots.size(); iw++) {
                    double detJ;
                    // Compute F= F(u,v,w)
                    ComputeF(Ulroots[iu], Vlroots[iv], Wlroots[iw], mF, state_x, state_w);
                    // Compute mNF= N(u,v,w)'*F
                    loadable->ComputeNF(Ulroots[iu], Vlroots[iv], Wlroots[iw], mNF, detJ, mF, state_x, state_w);
                    // Compute Q+= mNF * detJ * wi*wj*wk
                    mNF *= (detJ * Uweight[iu] * Vweight[iv] * Wweight[iw]);
                    Q += mNF;
                }
            }
        }
    }
}

void ChLoaderUVWatomic::ComputeQ(ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    // Compute F=F(u,v,w)
    ChVectorDynamic<> F(loadable->GetNumFieldCoords());
    F.setZero();
    ComputeF(Pu, Pv, Pw, F, state_x, state_w);

    // Compute N(u,v,w)'*F
    double detJ;
    Q.setZero(loadable->GetLoadableNumCoordsVelLevel());
    loadable->ComputeNF(Pu, Pv, Pw, Q, detJ, F, state_x, state_w);
}

void ChLoaderUVWatomic::SetApplication(double u, double v, double w) {
    Pu = u;
    Pv = v;
    Pw = w;
}

//--------------------------------------------------------------------------------

void ChLoaderGravity::ComputeF(double U,
                               double V,
                               double W,
                               ChVectorDynamic<>& F,
                               ChVectorDynamic<>* state_x,
                               ChVectorDynamic<>* state_w) {
    if ((F.size() == 3) || (F.size() == 6) || (F.size() == 9)) {
        // only for force or wrench fields
        F(0) = G_acc.x() * loadable->GetDensity();
        F(1) = G_acc.y() * loadable->GetDensity();
        F(2) = G_acc.z() * loadable->GetDensity();
    }
}

}  // end namespace chrono
