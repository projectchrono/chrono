// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Wrapper for a vehicle joint (kinematic or bushing)
//
// =============================================================================

#include "chrono_vehicle/ChVehicleJoint.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkUniversal.h"

namespace chrono {
namespace vehicle {

ChVehicleJoint::ChVehicleJoint(bool kinematic) : m_kinematic(kinematic), m_link{}, m_bushing{} {}
ChVehicleJoint::~ChVehicleJoint() {}

std::shared_ptr<ChVehicleJoint> ChVehicleJoint::Create(Type type,
                                                       const std::string& name,
                                                       std::shared_ptr<ChBody> body1,
                                                       std::shared_ptr<ChBody> body2,
                                                       ChCoordsys<> pos,
                                                       std::shared_ptr<ChVehicleBushingData> bushing_data) {
    bool kinematic = (bushing_data == nullptr);
    auto joint = chrono_types::make_shared<ChVehicleJoint>(kinematic);
    if (kinematic) {
        joint->CreateLink(type, body1, body2, pos);
        joint->m_link->SetNameString(name);
    } else {
        joint->CreateBushing(type, body1, body2, pos, bushing_data);
        joint->m_bushing->SetNameString(name);
    }
    return joint;
}

ChVector<> ChVehicleJoint::GetPos() const {
    if (m_kinematic) {
        return m_link->GetLinkAbsoluteCoords().pos;
    } else {
        return m_bushing->GetAbsoluteFrameB().coord.pos;
    }
}

ChVectorDynamic<> ChVehicleJoint::GetConstraintViolation() const {
    if (m_kinematic) {
        return m_link->GetC();
    } else {
        return ChVectorDynamic<>();
    }
}

std::shared_ptr<ChLink> ChVehicleJoint::GetAsLink() const {
    if (!m_kinematic)
        return nullptr;
    return m_link;
}

std::shared_ptr<ChLoadBodyBody> ChVehicleJoint::GetAsBushing() const {
    if (m_kinematic)
        return nullptr;
    return m_bushing;
}

void ChVehicleJoint::CreateLink(Type type,
                                std::shared_ptr<ChBody> body1,
                                std::shared_ptr<ChBody> body2,
                                ChCoordsys<> pos) {
    switch (type) {
        case Type::LOCK: {
            auto link = chrono_types::make_shared<ChLinkLockLock>();
            link->Initialize(body1, body2, pos);
            m_link = link;
            break;
        }
        case Type::SPHERICAL: {
            auto link = chrono_types::make_shared<ChLinkLockSpherical>();
            link->Initialize(body1, body2, pos);
            m_link = link;
            break;
        }
        case Type::REVOLUTE: {
            auto link = chrono_types::make_shared<ChLinkLockRevolute>();
            link->Initialize(body1, body2, pos);
            m_link = link;
            break;
        }
        case Type::UNIVERSAL: {
            auto link = chrono_types::make_shared<ChLinkUniversal>();
            link->Initialize(body1, body2, ChFrame<>(pos));
            m_link = link;
            break;
        }
    }
}

void ChVehicleJoint::CreateBushing(Type type,
                                   std::shared_ptr<ChBody> body1,
                                   std::shared_ptr<ChBody> body2,
                                   ChCoordsys<> pos,
                                   std::shared_ptr<ChVehicleBushingData> bd) {
    ChMatrixNM<double, 6, 6> K_matrix;
    ChMatrixNM<double, 6, 6> D_matrix;
    K_matrix.setZero();
    D_matrix.setZero();
    K_matrix.diagonal() << bd->K_lin, bd->K_lin, bd->K_lin, bd->K_rot, bd->K_rot, bd->K_rot;
    D_matrix.diagonal() << bd->D_lin, bd->D_lin, bd->D_lin, bd->D_rot, bd->D_rot, bd->D_rot;
    switch (type) {
        case Type::LOCK:
            break;
        case Type::SPHERICAL:
            K_matrix(3, 3) = bd->K_rot_dof;
            K_matrix(4, 4) = bd->K_rot_dof;
            K_matrix(5, 5) = bd->K_rot_dof;
            D_matrix(3, 3) = bd->D_rot_dof;
            D_matrix(4, 4) = bd->D_rot_dof;
            D_matrix(5, 5) = bd->D_rot_dof;
            break;
        case Type::REVOLUTE:
            K_matrix(5, 5) = bd->K_rot_dof;
            D_matrix(5, 5) = bd->D_rot_dof;
            break;
        case Type::UNIVERSAL:
            K_matrix(3, 3) = bd->K_rot_dof;
            K_matrix(4, 4) = bd->K_rot_dof;
            D_matrix(3, 3) = bd->D_rot_dof;
            D_matrix(4, 4) = bd->D_rot_dof;
            break;
    }

    m_bushing =
        chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(body1, body2, ChFrame<>(pos), K_matrix, D_matrix);
}

}  // namespace vehicle
}  // namespace chrono
