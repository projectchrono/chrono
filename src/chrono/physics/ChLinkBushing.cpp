// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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
// available to apply compliance, only those corresponsing the bushing type
// selected by the user are introduced.
//
// =============================================================================

#include "chrono/physics/ChLinkBushing.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and
// persistence
ChClassRegister<ChLinkBushing> a_registration_ChLinkLockBushing;

ChLinkBushing::ChLinkBushing(bushing_joint bushing_joint_type) {
    m_bushing_joint = bushing_joint_type;
    ChangeLinkType(LNK_FREE);  // Our bushing element will be a free joint
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

    // Bushing joint always applied compliance in all translation directions

    m_force_Tx = new ChLinkForce();
    m_force_Tx->Set_active(1);
    m_force_Tx->Set_K(m_constants_K(0, 0));
    m_force_Tx->Set_R(m_constants_R(0, 0));
    SetForce_X(m_force_Tx);

    m_force_Ty = new ChLinkForce();
    m_force_Ty->Set_active(1);
    m_force_Ty->Set_K(m_constants_K(1, 1));
    m_force_Ty->Set_R(m_constants_R(1, 1));
    SetForce_Y(m_force_Ty);

    m_force_Tz = new ChLinkForce();
    m_force_Tz->Set_active(1);
    m_force_Tz->Set_K(m_constants_K(2, 2));
    m_force_Tz->Set_R(m_constants_R(2, 2));
    SetForce_Z(m_force_Tz);

    // Apply compliance in rotational dofs depending on bushing type selected
    switch (m_bushing_joint) {
        case ChLinkBushing::Mount:
            m_force_Rx = new ChLinkForce();
            m_force_Rx->Set_active(1);
            m_force_Rx->Set_K(m_constants_K(3, 3));
            m_force_Rx->Set_R(m_constants_R(3, 3));
            SetForce_Rx(m_force_Rx);

            m_force_Ry = new ChLinkForce();
            m_force_Ry->Set_active(1);
            m_force_Ry->Set_K(m_constants_K(4, 4));
            m_force_Ry->Set_R(m_constants_R(4, 4));
            SetForce_Ry(m_force_Ry);

            m_force_Rz = new ChLinkForce();
            m_force_Rz->Set_active(1);
            m_force_Rz->Set_K(m_constants_K(5, 5));
            m_force_Rz->Set_R(m_constants_R(5, 5));
            SetForce_Rz(m_force_Rz);

            break;
        case ChLinkBushing::Spherical:
            break;
        case ChLinkBushing::Revolute:
            m_force_Rx = new ChLinkForce();
            m_force_Rx->Set_active(1);
            m_force_Rx->Set_K(m_constants_K(3, 3));
            m_force_Rx->Set_R(m_constants_R(3, 3));
            SetForce_Rx(m_force_Rx);

            m_force_Ry = new ChLinkForce();
            m_force_Ry->Set_active(1);
            m_force_Ry->Set_K(m_constants_K(4, 4));
            m_force_Ry->Set_R(m_constants_R(4, 4));
            SetForce_Ry(m_force_Ry);
            break;
    }
    return;
}
void ChLinkBushing::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLinkLock::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(m_force_Rx);
    marchive << CHNVP(m_force_Ry);
    marchive << CHNVP(m_force_Rz);
    marchive << CHNVP(m_force_Tx);
    marchive << CHNVP(m_force_Ty);
    marchive << CHNVP(m_force_Tz);
    marchive << CHNVP(m_constants_K);
    marchive << CHNVP(m_constants_R);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkBushing::ArchiveIN(ChArchiveIn& marchive) {
    // deserialize parent class
    ChLinkLock::ArchiveIN(marchive);

    // serialize all member data:
    marchive >> CHNVP(m_force_Rx);
    marchive >> CHNVP(m_force_Ry);
    marchive >> CHNVP(m_force_Rz);
    marchive >> CHNVP(m_force_Tx);
    marchive >> CHNVP(m_force_Ty);
    marchive >> CHNVP(m_force_Tz);
    marchive >> CHNVP(m_constants_K);
    marchive >> CHNVP(m_constants_R);
}

}  // end namespace chrono
