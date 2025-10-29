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


#include "chrono/fea/ChFieldElement.h"
#include "ChFieldElementLoadableVolume.h"

namespace chrono {
namespace fea {



/// Evaluate N'*F , where N is some type of shape function evaluated at U,V coordinates of the surface, each ranging
/// in -1..+1 F is a load, N'*F is the resulting generalized load. Returns also det[J] with J=[dx/du,..], which may
/// be useful in Gauss quadrature.

inline void fea::ChFieldElementLoadableVolume::ComputeNF(const double U, const double V, const double W, ChVectorDynamic<>& Qi, double& detJ, const ChVectorDynamic<>& F, ChVectorDynamic<>* state_x, ChVectorDynamic<>* state_w) {
    ChRowVectorDynamic<double> N(m_element->GetNumNodes());
    ChVector3d eta(U, V, W);
    m_element->ComputeN(eta, N);

    ChMatrix33<double> J;

    detJ = m_element->ComputeJ(eta, J);

    int ncoords_node = m_field->GetNumFieldCoordsVelLevel();
    for (unsigned int i = 0; i < m_element->GetNumNodes(); i++) {
        Qi.segment(ncoords_node * i, ncoords_node) = N(i) * F.segment(0, ncoords_node);
    }
}

}  // end namespace fea
}  // end namespace chrono
