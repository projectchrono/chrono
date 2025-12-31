// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Wrapper for a joint (kinematic or bushing)
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChJoint.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkUniversal.h"

namespace chrono {

ChJoint::ChJoint(Type type,
                 const std::string& name,
                 std::shared_ptr<ChBody> body1,
                 std::shared_ptr<ChBody> body2,
                 ChFrame<> joint_frame,
                 std::shared_ptr<BushingData> bushing_data) {
    if (bushing_data == nullptr) {
        CreateLink(type, body1, body2, joint_frame);
        mpark::get<Link>(m_joint)->SetName(name);
    } else {
        CreateBushing(type, body1, body2, joint_frame, bushing_data);
        mpark::get<Bushing>(m_joint)->SetName(name);
    }
}

ChJoint::~ChJoint() {}

void ChJoint::SetTag(int tag) {
    if (m_joint.index() == 0) {
        mpark::get<Link>(m_joint)->SetTag(tag);
    } else {
        mpark::get<Bushing>(m_joint)->SetTag(tag);
    }
}

bool ChJoint::IsKinematic() const {
    return m_joint.index() == 0;
}

ChVector3d ChJoint::GetPos() const {
    if (m_joint.index() == 0) {
        return mpark::get<Link>(m_joint)->GetFrame2Abs().GetCoordsys().pos;
    } else {
        return mpark::get<Bushing>(m_joint)->GetAbsoluteFrameB().GetPos();
    }
}

ChVectorDynamic<> ChJoint::GetConstraintViolation() const {
    if (m_joint.index() == 0) {
        return mpark::get<Link>(m_joint)->GetConstraintViolation();
    } else {
        return ChVectorDynamic<>();
    }
}

ChVector3d ChJoint::GetForce() const {
    if (m_joint.index() == 0) {
        return mpark::get<Link>(m_joint)->GetReaction2().force;
    } else {
        return mpark::get<Bushing>(m_joint)->GetForce();
    }
}

ChJoint::Link ChJoint::GetAsLink() const {
    return *mpark::get_if<Link>(&m_joint);
}

ChJoint::Bushing ChJoint::GetAsBushing() const {
    return *mpark::get_if<Bushing>(&m_joint);
}

void ChJoint::CreateLink(Type type,
                         std::shared_ptr<ChBody> body1,
                         std::shared_ptr<ChBody> body2,
                         ChFrame<> link_frame) {
    switch (type) {
        case Type::LOCK: {
            auto link = chrono_types::make_shared<ChLinkLockLock>();
            link->Initialize(body1, body2, link_frame);
            m_joint = link;
            break;
        }
        case Type::SPHERICAL: {
            auto link = chrono_types::make_shared<ChLinkLockSpherical>();
            link->Initialize(body1, body2, link_frame);
            m_joint = link;
            break;
        }
        case Type::REVOLUTE: {
            auto link = chrono_types::make_shared<ChLinkLockRevolute>();
            link->Initialize(body1, body2, link_frame);
            m_joint = link;
            break;
        }
        case Type::PRISMATIC: {
            auto link = chrono_types::make_shared<ChLinkLockPrismatic>();
            link->Initialize(body1, body2, link_frame);
            m_joint = link;
            break;
        }
        case Type::UNIVERSAL: {
            auto link = chrono_types::make_shared<ChLinkUniversal>();
            link->Initialize(body1, body2, link_frame);
            m_joint = link;
            break;
        }
        case Type::POINTLINE: {
            auto link = chrono_types::make_shared<ChLinkLockPointLine>();
            link->Initialize(body1, body2, link_frame);
            m_joint = link;
            break;
        }
        case Type::POINTPLANE: {
            auto link = chrono_types::make_shared<ChLinkLockPointPlane>();
            link->Initialize(body1, body2, link_frame);
            m_joint = link;
            break;
        }
    }
}

void ChJoint::CreateBushing(Type type,
                            std::shared_ptr<ChBody> body1,
                            std::shared_ptr<ChBody> body2,
                            ChFrame<> bushing_frame,
                            std::shared_ptr<BushingData> bd) {
    ChMatrix66d K_matrix;
    ChMatrix66d D_matrix;
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
        case Type::PRISMATIC:
        case Type::POINTLINE:
        case Type::POINTPLANE:
            return;  // do not create a bushing
    }

    m_joint = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(body1, body2, bushing_frame, K_matrix, D_matrix);
}

std::string ChJoint::GetTypeString(Type type) {
    switch (type) {
        case Type::LOCK:
            return "lock";
        case Type::SPHERICAL:
            return "spherical";
        case Type::REVOLUTE:
            return "revolute";
        case Type::PRISMATIC:
            return "prismatic";
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

void ChJoint::Remove(std::shared_ptr<ChJoint> joint) {
    if (joint->m_joint.index() == 0) {
        ChJoint::Link& link = mpark::get<ChJoint::Link>(joint->m_joint);
        auto sys = link->GetSystem();
        if (sys) {
            sys->Remove(link);
        }
    }
    // Note: bushing are removed when the load container is removed
}

}  // namespace chrono
