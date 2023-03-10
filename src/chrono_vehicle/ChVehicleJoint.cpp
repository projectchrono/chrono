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

ChVehicleJoint::ChVehicleJoint(Type type,
                               const std::string& name,
                               std::shared_ptr<ChBody> body1,
                               std::shared_ptr<ChBody> body2,
                               ChCoordsys<> pos,
                               std::shared_ptr<ChVehicleBushingData> bushing_data) {
    if (bushing_data == nullptr) {
        CreateLink(type, body1, body2, pos);
        mpark::get<Link>(m_joint)->SetNameString(name);
    } else {
        CreateBushing(type, body1, body2, pos, bushing_data);
        mpark::get<Bushing>(m_joint)->SetNameString(name);
    }
}

ChVehicleJoint::~ChVehicleJoint() {}

bool ChVehicleJoint::IsKinematic() const {
    return m_joint.index() == 0;
}

ChVector<> ChVehicleJoint::GetPos() const {
    if (m_joint.index() == 0) {
        return mpark::get<Link>(m_joint)->GetLinkAbsoluteCoords().pos;
    } else {
        return mpark::get<Bushing>(m_joint)->GetAbsoluteFrameB().coord.pos;
    }
}

ChVectorDynamic<> ChVehicleJoint::GetConstraintViolation() const {
    if (m_joint.index() == 0) {
        return mpark::get<Link>(m_joint)->GetConstraintViolation();
    } else {
        return ChVectorDynamic<>();
    }
}

ChVector<> ChVehicleJoint::GetForce() const {
    if (m_joint.index() == 0) {
        return mpark::get<Link>(m_joint)->Get_react_force();
    } else {
        return mpark::get<Bushing>(m_joint)->GetForce();
    }
}

ChVehicleJoint::Link ChVehicleJoint::GetAsLink() const {
    return *mpark::get_if<Link>(&m_joint);
}

ChVehicleJoint::Bushing ChVehicleJoint::GetAsBushing() const {
    return *mpark::get_if<Bushing>(&m_joint);
}

void ChVehicleJoint::CreateLink(Type type,
                                std::shared_ptr<ChBody> body1,
                                std::shared_ptr<ChBody> body2,
                                ChCoordsys<> pos) {
    switch (type) {
        case Type::LOCK: {
            auto link = chrono_types::make_shared<ChLinkLockLock>();
            link->Initialize(body1, body2, pos);
            m_joint = link;
            break;
        }
        case Type::SPHERICAL: {
            auto link = chrono_types::make_shared<ChLinkLockSpherical>();
            link->Initialize(body1, body2, pos);
            m_joint = link;
            break;
        }
        case Type::REVOLUTE: {
            auto link = chrono_types::make_shared<ChLinkLockRevolute>();
            link->Initialize(body1, body2, pos);
            m_joint = link;
            break;
        }
        case Type::UNIVERSAL: {
            auto link = chrono_types::make_shared<ChLinkUniversal>();
            link->Initialize(body1, body2, ChFrame<>(pos));
            m_joint = link;
            break;
        }
        case Type::POINTLINE: {
            auto link = chrono_types::make_shared<ChLinkLockPointLine>();
            link->Initialize(body1, body2, pos);
            m_joint = link;
            break;
        }
        case Type::POINTPLANE: {
            auto link = chrono_types::make_shared<ChLinkLockPointPlane>();
            link->Initialize(body1, body2, pos);
            m_joint = link;
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
        case Type::POINTLINE:
        case Type::POINTPLANE:
            return; // do not create a bushing
    }

    m_joint = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(body1, body2, ChFrame<>(pos), K_matrix, D_matrix);
}

std::string ChVehicleJoint::GetTypeString(Type type) {
    switch (type) {
        case Type::LOCK:
            return "lock";
        case Type::SPHERICAL:
            return "spherical";
        case Type::REVOLUTE:
            return "revolute";
        case Type::UNIVERSAL:
            return "universal";
        case Type::POINTLINE:
            return "point-line";
        case Type::POINTPLANE:
            return "point-plane";
        default:
            return "unknown";
    }
}

}  // namespace vehicle
}  // namespace chrono
