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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChLoadsXYZnode.h"

namespace chrono {

// -----------------------------------------------------------------------------
// ChLoadXYZnodeForce
// -----------------------------------------------------------------------------

ChLoadXYZnodeForce::ChLoadXYZnodeForce(std::shared_ptr<ChNodeXYZ> node) : ChLoadCustom(node) {
    computed_abs_force = VNULL;
}

void ChLoadXYZnodeForce::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto node = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadable);

    ChVector<> nodeApos;
    ChVector<> nodeApos_dt;

    if (state_x) {
        // the numerical jacobian algo might change state_x
        nodeApos = ChVector<>(state_x->segment(0, 3));
    } else {
        nodeApos = node->GetPos();
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        nodeApos_dt = ChVector<>(state_w->segment(0, 3));
    } else {
        nodeApos_dt = node->GetPos_dt();
    }

    ComputeForce(nodeApos, nodeApos_dt, computed_abs_force);

    // Compute Q
    load_Q.segment(0, 3) = computed_abs_force.eigen();
}

void ChLoadXYZnodeForce::Update(double time) {
    ChLoadCustom::Update(time);
}

// -----------------------------------------------------------------------------
// ChLoadXYZnodeForceAbsolute
// -----------------------------------------------------------------------------

ChLoadXYZnodeForceAbsolute::ChLoadXYZnodeForceAbsolute(std::shared_ptr<ChNodeXYZ> body, const ChVector<>& force)
    : ChLoadXYZnodeForce(body), m_force_base(force), m_scale(1) {
    m_modulation = chrono_types::make_shared<ChFunction_Const>(1.0);
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeForceAbsolute::ComputeForce(const ChVector<>& abs_pos,
                                              const ChVector<>& abs_vel,
                                              ChVector<>& abs_force) {
    abs_force = GetForce();
}

void ChLoadXYZnodeForceAbsolute::Update(double time) {
    m_modulation->Update(time);
    m_scale = m_modulation->Get_y(time);
    ChLoadXYZnodeForce::Update(time);
}

void ChLoadXYZnodeForceAbsolute::SetForceBase(const ChVector<>& force) {
    m_force_base = force;
}

ChVector<> ChLoadXYZnodeForceAbsolute::GetForce() const {
    return m_force_base * m_scale;
}

// -----------------------------------------------------------------------------
// ChLoadXYZnodeXYZnode
// -----------------------------------------------------------------------------

ChLoadXYZnodeXYZnode::ChLoadXYZnodeXYZnode(std::shared_ptr<ChNodeXYZ> nodeA, std::shared_ptr<ChNodeXYZ> nodeB)
    : ChLoadCustomMultiple(nodeA, nodeB) {
    computed_abs_force = VNULL;
}

void ChLoadXYZnodeXYZnode::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mnodeA = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[0]);
    auto mnodeB = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[1]);

    ChVector<> nodeApos;
    ChVector<> nodeApos_dt;
    ChVector<> nodeBpos;
    ChVector<> nodeBpos_dt;

    if (state_x) {
        // the numerical jacobian algo might change state_x
        nodeApos = state_x->segment(0, 3);
        nodeBpos = state_x->segment(3, 3);
    } else {
        nodeApos = mnodeA->GetPos();
        nodeBpos = mnodeB->GetPos();
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        nodeApos_dt = state_w->segment(0, 3);
        nodeBpos_dt = state_w->segment(3, 3);
    } else {
        nodeApos_dt = mnodeA->GetPos_dt();
        nodeBpos_dt = mnodeB->GetPos_dt();
    }

    ComputeForce((nodeApos - nodeBpos), (nodeApos_dt - nodeBpos_dt), computed_abs_force);

    // Compute Q
    load_Q.segment(0, 3) = computed_abs_force.eigen();
    load_Q.segment(3, 3) = -computed_abs_force.eigen();
}

void ChLoadXYZnodeXYZnode::Update(double time) {
    ChLoadCustomMultiple::Update(time);
}

// -----------------------------------------------------------------------------
// ChLoadXYZnodeXYZnodeSpring
// -----------------------------------------------------------------------------

ChLoadXYZnodeXYZnodeSpring::ChLoadXYZnodeXYZnodeSpring(std::shared_ptr<ChNodeXYZ> nodeA,
                                                       std::shared_ptr<ChNodeXYZ> nodeB,
                                                       double stiffness,
                                                       double damping,
                                                       double rest_length)
    : ChLoadXYZnodeXYZnode(nodeA, nodeB), K(stiffness), R(damping), d0(rest_length), is_stiff(false) {}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeXYZnodeSpring::ComputeForce(const ChVector<>& rel_pos,
                                              const ChVector<>& rel_vel,
                                              ChVector<>& abs_force) {
    ChVector<> BA = rel_pos.GetNormalized();
    double d = rel_pos.Length() - d0;
    double d_dt = Vdot(rel_vel, BA);
    abs_force = (-K * d - R * d_dt) * BA;
}

// -----------------------------------------------------------------------------
// ChLoadXYZnodeXYZnodeBushing
// -----------------------------------------------------------------------------

ChLoadXYZnodeXYZnodeBushing::ChLoadXYZnodeXYZnodeBushing(std::shared_ptr<ChNodeXYZ> nodeA,
                                                         std::shared_ptr<ChNodeXYZ> nodeB)
    : ChLoadXYZnodeXYZnode(nodeA, nodeB), R(VNULL), is_stiff(false) {
    force_dX = chrono_types::make_shared<ChFunction_Const>(0.0);
    force_dY = chrono_types::make_shared<ChFunction_Const>(0.0);
    force_dZ = chrono_types::make_shared<ChFunction_Const>(0.0);
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeXYZnodeBushing::ComputeForce(const ChVector<>& rel_pos,
                                               const ChVector<>& rel_vel,
                                               ChVector<>& abs_force) {
    abs_force = ChVector<>(force_dX->Get_y(rel_pos.x()) - R.x() * rel_vel.x(),
                           force_dY->Get_y(rel_pos.y()) - R.y() * rel_vel.y(),
                           force_dZ->Get_y(rel_pos.z()) - R.z() * rel_vel.z());
}

// -----------------------------------------------------------------------------
// ChLoadBodyBody
// -----------------------------------------------------------------------------

ChLoadXYZnodeBody::ChLoadXYZnodeBody(std::shared_ptr<ChNodeXYZ> nodeA, std::shared_ptr<ChBody> bodyB)
    : ChLoadCustomMultiple(nodeA, bodyB) {
    ChFrame<> abs_application(nodeA->GetPos());
    bodyB->ChFrame::TransformParentToLocal(abs_application, loc_application_B);
    computed_loc_force = VNULL;
}

void ChLoadXYZnodeBody::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto nodeA = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[0]);
    auto bodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);

    ChFrameMoving<> bodycoordA, bodycoordB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        bodycoordA.SetPos(state_x->segment(0, 3));
        bodycoordB.SetCoord(state_x->segment(3, 7));
    } else {
        bodycoordA.SetPos(nodeA->pos);
        bodycoordB.SetCoord(bodyB->coord);
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        bodycoordA.SetPos_dt(state_w->segment(0, 3));
        bodycoordB.SetPos_dt(state_w->segment(3, 3));
        bodycoordB.SetWvel_loc(state_w->segment(6, 3));
    } else {
        bodycoordA.SetPos_dt(nodeA->GetPos_dt());
        bodycoordB.SetCoord_dt(bodyB->GetCoord_dt());
    }

    frame_Aw = bodycoordA;
    frame_Bw = ChFrameMoving<>(loc_application_B) >> bodycoordB;
    ChFrameMoving<> rel_AB = frame_Aw >> frame_Bw.GetInverse();

    // COMPUTE THE FORCE

    ComputeForce(rel_AB, computed_loc_force);

    ChVector<> abs_force = frame_Bw.TransformDirectionLocalToParent(computed_loc_force);

    // Compute Q
    load_Q.segment(0, 3) = abs_force.eigen();

    ChVector<> loc_ftorque = bodycoordB.GetRot().RotateBack(((frame_Bw.GetPos() - bodycoordB.GetPos()) % abs_force));
    load_Q.segment(3, 3) = -abs_force.eigen();
    load_Q.segment(6, 3) = -loc_ftorque.eigen();
}

std::shared_ptr<ChNodeXYZ> ChLoadXYZnodeBody::GetNodeA() const {
    return std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[0]);
}

std::shared_ptr<ChBody> ChLoadXYZnodeBody::GetBodyB() const {
    return std::dynamic_pointer_cast<ChBody>(this->loadables[1]);
}

// -----------------------------------------------------------------------------
// ChLoadXYZnodeBodySpring
// -----------------------------------------------------------------------------

ChLoadXYZnodeBodySpring::ChLoadXYZnodeBodySpring(std::shared_ptr<ChNodeXYZ> nodeA,
                                                 std::shared_ptr<ChBody> bodyB,
                                                 double stiffness,
                                                 double damping,
                                                 double rest_length)
    : ChLoadXYZnodeBody(nodeA, bodyB), K(stiffness), R(damping), d0(rest_length) {
    is_stiff = false;
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeBodySpring::ComputeForce(const ChFrameMoving<>& rel_AB, ChVector<>& loc_force) {
    ChVector<> BA = rel_AB.GetPos().GetNormalized();
    double d = rel_AB.GetPos().Length() - d0;
    double d_dt = Vdot(rel_AB.GetPos_dt(), BA);
    loc_force = (-K * d - R * d_dt) * BA;
}

// -----------------------------------------------------------------------------
// ChLoadXYZnodeBodyBushing
// -----------------------------------------------------------------------------

ChLoadXYZnodeBodyBushing::ChLoadXYZnodeBodyBushing(std::shared_ptr<ChNodeXYZ> nodeA, std::shared_ptr<ChBody> bodyB)
    : ChLoadXYZnodeBody(nodeA, bodyB) {
    force_dX = chrono_types::make_shared<ChFunction_Const>(0.0);
    force_dY = chrono_types::make_shared<ChFunction_Const>(0.0);
    force_dZ = chrono_types::make_shared<ChFunction_Const>(0.0);
    R = VNULL;
    is_stiff = false;
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeBodyBushing::ComputeForce(const ChFrameMoving<>& rel_AB, ChVector<>& loc_force) {
    loc_force = ChVector<>(force_dX->Get_y(rel_AB.GetPos().x()) - R.x() * rel_AB.GetPos_dt().x(),
                           force_dY->Get_y(rel_AB.GetPos().y()) - R.y() * rel_AB.GetPos_dt().y(),
                           force_dZ->Get_y(rel_AB.GetPos().z()) - R.z() * rel_AB.GetPos_dt().z());
}

}  // end namespace chrono
