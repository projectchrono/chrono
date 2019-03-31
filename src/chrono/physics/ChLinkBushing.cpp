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
// Authors: Antonio Recuero
//
// Class that inherits from ChLinkLock as a free joint. Compliances are added
// to the relative motion between two rigid bodies. Out of the 6 possible dofs
// available to apply compliance, only those corresponding the bushing type
// selected by the user are introduced.
//
// =============================================================================

#include "chrono/physics/ChLinkBushing.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and
// persistence
CH_FACTORY_REGISTER(ChLinkBushing)

ChLinkBushing::ChLinkBushing(bushing_joint bushing_joint_type) {
    m_bushing_joint = bushing_joint_type;
    ChangeLinkType(LinkType::FREE);  // Our bushing element will be a free joint
}

ChLinkBushing::~ChLinkBushing() {}

void ChLinkBushing::Initialize(std::shared_ptr<ChBody> mbody1,
                               std::shared_ptr<ChBody> mbody2,
                               const ChCoordsys<>& mpos,
                               const ChMatrixNM<double, 6, 6>& K,
                               const ChMatrixNM<double, 6, 6>& R) {
    ChLinkMarkers::Initialize(mbody1, mbody2, mpos);
    m_constants_K = K;  // K_x, K_y, K_z, KTheta_x, KTheta_y, KTheta_z
    m_constants_R = R;  // R_x, R_y, R_z, RTheta_x, RTheta_y, RTheta_z

    // Bushing joint always applied compliance in all translation directions.
    // Use force objects (force_X, force_Y, force_Z; force_Rx, force_Ry, force_Rz) from parent class.

    force_X = std::make_unique<ChLinkForce>();
    force_X->SetActive(true);
    force_X->SetK(m_constants_K(0, 0));
    force_X->SetR(m_constants_R(0, 0));

    force_Y = std::make_unique<ChLinkForce>();
    force_Y->SetActive(true);
    force_Y->SetK(m_constants_K(1, 1));
    force_Y->SetR(m_constants_R(1, 1));

    force_Z = std::make_unique<ChLinkForce>();
    force_Z->SetActive(true);
    force_Z->SetK(m_constants_K(2, 2));
    force_Z->SetR(m_constants_R(2, 2));

    // Apply compliance in rotational dofs depending on bushing type selected
    switch (m_bushing_joint) {
        case ChLinkBushing::Mount:
            force_Rx = std::make_unique<ChLinkForce>();
            force_Rx->SetActive(true);
            force_Rx->SetK(m_constants_K(3, 3));
            force_Rx->SetR(m_constants_R(3, 3));

            force_Ry = std::make_unique<ChLinkForce>();
            force_Ry->SetActive(true);
            force_Ry->SetK(m_constants_K(4, 4));
            force_Ry->SetR(m_constants_R(4, 4));

            force_Rz = std::make_unique<ChLinkForce>();
            force_Rz->SetActive(true);
            force_Rz->SetK(m_constants_K(5, 5));
            force_Rz->SetR(m_constants_R(5, 5));

            break;
        case ChLinkBushing::Spherical:
            break;
    }
    return;
}
void ChLinkBushing::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkBushing>();

    // serialize parent class
    ChLinkLock::ArchiveOUT(marchive);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkBushing::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    marchive.VersionRead<ChLinkBushing>();

    // deserialize parent class
    ChLinkLock::ArchiveIN(marchive);
}

}  // end namespace chrono
