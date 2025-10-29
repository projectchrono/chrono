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


#include "chrono/fea/ChLoaderHeatConvection.h"

namespace chrono {
namespace fea {


inline void ChLoaderHeatConvection::ComputeF(double U, double V, ChVectorDynamic<>& F, ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    if (auto face = std::dynamic_pointer_cast<ChFieldElement>(this->loadable)) {
        if (face->GetManifoldDimensions() == 2) {

            // Ck, we are loading a "face" finite element, so we can use NodeData(node) and fetch associated temperature .
            // Compute shape functions for interpolating temperature from the m_temp auxiliary field:
            ChRowVectorDynamic<> N(face->GetNumNodes());
            face->ComputeN(ChVector3d(U, V, 0), N);
            // Interpolate T as  T = [T1,T2,T3,...] * N^T
            ChRowVectorDynamic<> T_hat(face->GetNumNodes());
            for (unsigned int in = 0; in < face->GetNumNodes(); ++in) {
                T_hat(in) = m_temp->NodeData(face->GetNode(in)).T();
            }
            double T = T_hat * N.transpose();

            // Compute the heat flux using the convection model W_per_area = convection_coeff * (T_surface - T_fluid):
            F(0) = -this->m_convection_coeff * (T - m_T_fluid);
        }
    }
    F(0) = 0; // fallback if no face 
}

}  // end namespace fea
}  // end namespace chrono
