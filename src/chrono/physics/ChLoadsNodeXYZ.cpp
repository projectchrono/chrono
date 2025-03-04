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

#include "chrono/physics/ChLoadsNodeXYZ.h"

namespace chrono {

// -----------------------------------------------------------------------------
// ChLoadNodeXYZForce
// -----------------------------------------------------------------------------

ChLoadNodeXYZForce::ChLoadNodeXYZForce(std::shared_ptr<ChNodeXYZ> node) : ChLoadCustom(node) {
    computed_abs_force = VNULL;
}

void ChLoadNodeXYZForce::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto node = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadable);

    ChVector3d nodeApos;
    ChVector3d nodeApos_dt;

    if (state_x) {
        // the numerical jacobian algo might change state_x
        nodeApos = ChVector3d(state_x->segment(0, 3));
    } else {
        nodeApos = node->GetPos();
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        nodeApos_dt = ChVector3d(state_w->segment(0, 3));
    } else {
        nodeApos_dt = node->GetPosDt();
    }

    ComputeForce(nodeApos, nodeApos_dt, computed_abs_force);

    // Compute Q
    load_Q.segment(0, 3) = computed_abs_force.eigen();
}

void ChLoadNodeXYZForce::Update(double time, bool update_assets) {
    ChLoadCustom::Update(time, update_assets);
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZForceAbs
// -----------------------------------------------------------------------------

ChLoadNodeXYZForceAbs::ChLoadNodeXYZForceAbs(std::shared_ptr<ChNodeXYZ> body, const ChVector3d& force)
    : ChLoadNodeXYZForce(body), m_force_base(force), m_scale(1) {
    m_modulation = chrono_types::make_shared<ChFunctionConst>(1.0);
}

// Compute the force on the node, in absolute coordsystem, given position of node as abs_pos.
void ChLoadNodeXYZForceAbs::ComputeForce(const ChVector3d& abs_pos, const ChVector3d& abs_vel, ChVector3d& abs_force) {
    abs_force = GetForce();
}

void ChLoadNodeXYZForceAbs::Update(double time, bool update_assets) {
    m_modulation->Update(time);
    m_scale = m_modulation->GetVal(time);
    ChLoadNodeXYZForce::Update(time, update_assets);
}

void ChLoadNodeXYZForceAbs::SetForceBase(const ChVector3d& force) {
    m_force_base = force;
}

ChVector3d ChLoadNodeXYZForceAbs::GetForce() const {
    return m_force_base * m_scale;
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZNodeXYZ
// -----------------------------------------------------------------------------

ChLoadNodeXYZNodeXYZ::ChLoadNodeXYZNodeXYZ(std::shared_ptr<ChNodeXYZ> nodeA, std::shared_ptr<ChNodeXYZ> nodeB)
    : ChLoadCustomMultiple(nodeA, nodeB) {
    computed_abs_force = VNULL;
}

void ChLoadNodeXYZNodeXYZ::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mnodeA = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[0]);
    auto mnodeB = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[1]);

    ChVector3d nodeApos;
    ChVector3d nodeApos_dt;
    ChVector3d nodeBpos;
    ChVector3d nodeBpos_dt;

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
        nodeApos_dt = mnodeA->GetPosDt();
        nodeBpos_dt = mnodeB->GetPosDt();
    }

    ComputeForce((nodeApos - nodeBpos), (nodeApos_dt - nodeBpos_dt), computed_abs_force);

    // Compute Q
    load_Q.segment(0, 3) = computed_abs_force.eigen();
    load_Q.segment(3, 3) = -computed_abs_force.eigen();
}

void ChLoadNodeXYZNodeXYZ::Update(double time, bool update_assets) {
    ChLoadCustomMultiple::Update(time, update_assets);
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZNodeXYZSpring
// -----------------------------------------------------------------------------

ChLoadNodeXYZNodeXYZSpring::ChLoadNodeXYZNodeXYZSpring(std::shared_ptr<ChNodeXYZ> nodeA,
                                                       std::shared_ptr<ChNodeXYZ> nodeB,
                                                       double stiffness,
                                                       double damping,
                                                       double rest_length)
    : ChLoadNodeXYZNodeXYZ(nodeA, nodeB), K(stiffness), R(damping), d0(rest_length), is_stiff(false) {}

// Compute the force on the node, in absolute coordsystem, given position of node as abs_pos.
void ChLoadNodeXYZNodeXYZSpring::ComputeForce(const ChVector3d& rel_pos,
                                              const ChVector3d& rel_vel,
                                              ChVector3d& abs_force) {
    ChVector3d BA = rel_pos.GetNormalized();
    double d = rel_pos.Length() - d0;
    double d_dt = Vdot(rel_vel, BA);
    abs_force = (-K * d - R * d_dt) * BA;
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZNodeXYZBushing
// -----------------------------------------------------------------------------

ChLoadNodeXYZNodeXYZBushing::ChLoadNodeXYZNodeXYZBushing(std::shared_ptr<ChNodeXYZ> nodeA,
                                                         std::shared_ptr<ChNodeXYZ> nodeB)
    : ChLoadNodeXYZNodeXYZ(nodeA, nodeB), R(VNULL), is_stiff(false) {
    force_dX = chrono_types::make_shared<ChFunctionConst>(0.0);
    force_dY = chrono_types::make_shared<ChFunctionConst>(0.0);
    force_dZ = chrono_types::make_shared<ChFunctionConst>(0.0);
}

// Compute the force on the node, in absolute coordsystem, given position of node as abs_pos.
void ChLoadNodeXYZNodeXYZBushing::ComputeForce(const ChVector3d& rel_pos,
                                               const ChVector3d& rel_vel,
                                               ChVector3d& abs_force) {
    abs_force = ChVector3d(force_dX->GetVal(rel_pos.x()) - R.x() * rel_vel.x(),
                           force_dY->GetVal(rel_pos.y()) - R.y() * rel_vel.y(),
                           force_dZ->GetVal(rel_pos.z()) - R.z() * rel_vel.z());
}

// -----------------------------------------------------------------------------
// ChLoadBodyBody
// -----------------------------------------------------------------------------

ChLoadNodeXYZBody::ChLoadNodeXYZBody(std::shared_ptr<ChNodeXYZ> node, std::shared_ptr<ChBody> body)
    : ChLoadCustomMultiple(node, body) {
    ChFrame<> abs_application(node->GetPos());
    loc_application_B = body->ChFrame::TransformParentToLocal(abs_application);
    computed_loc_force = VNULL;
}

void ChLoadNodeXYZBody::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto nodeA = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[0]);
    auto bodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);

    ChFrameMoving<> bodycoordA, bodycoordB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        bodycoordA.SetPos(state_x->segment(0, 3));
        bodycoordB.SetCoordsys(state_x->segment(3, 7));
    } else {
        bodycoordA.SetPos(nodeA->pos);
        bodycoordB.SetCoordsys(bodyB->GetCoordsys());
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        bodycoordA.SetPosDt(state_w->segment(0, 3));
        bodycoordB.SetPosDt(state_w->segment(3, 3));
        bodycoordB.SetAngVelLocal(state_w->segment(6, 3));
    } else {
        bodycoordA.SetPosDt(nodeA->GetPosDt());
        bodycoordB.SetCoordsysDt(bodyB->GetCoordsysDt());
    }

    frame_Aw = bodycoordA;
    frame_Bw = ChFrameMoving<>(loc_application_B) >> bodycoordB;
    ChFrameMoving<> rel_AB = frame_Aw >> frame_Bw.GetInverse();

    // COMPUTE THE FORCE

    ComputeForce(rel_AB, computed_loc_force);

    ChVector3d abs_force = frame_Bw.TransformDirectionLocalToParent(computed_loc_force);

    // Compute Q
    load_Q.segment(0, 3) = abs_force.eigen();

    ChVector3d loc_ftorque = bodycoordB.GetRot().RotateBack(((frame_Bw.GetPos() - bodycoordB.GetPos()) % abs_force));
    load_Q.segment(3, 3) = -abs_force.eigen();
    load_Q.segment(6, 3) = -loc_ftorque.eigen();
}

std::shared_ptr<ChNodeXYZ> ChLoadNodeXYZBody::GetNode() const {
    return std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[0]);
}

std::shared_ptr<ChBody> ChLoadNodeXYZBody::GetBody() const {
    return std::dynamic_pointer_cast<ChBody>(this->loadables[1]);
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZBodySpring
// -----------------------------------------------------------------------------

ChLoadNodeXYZBodySpring::ChLoadNodeXYZBodySpring(std::shared_ptr<ChNodeXYZ> nodeA,
                                                 std::shared_ptr<ChBody> bodyB,
                                                 double stiffness,
                                                 double damping,
                                                 double rest_length)
    : ChLoadNodeXYZBody(nodeA, bodyB), K(stiffness), R(damping), d0(rest_length) {
    is_stiff = false;
}

// Compute the force on the node, in absolute coordsystem, given position of node as abs_pos.
void ChLoadNodeXYZBodySpring::ComputeForce(const ChFrameMoving<>& rel_AB, ChVector3d& loc_force) {
    ChVector3d BA = rel_AB.GetPos().GetNormalized();
    double d = rel_AB.GetPos().Length() - d0;
    double d_dt = Vdot(rel_AB.GetPosDt(), BA);
    loc_force = (-K * d - R * d_dt) * BA;
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZBodyBushing
// -----------------------------------------------------------------------------

ChLoadNodeXYZBodyBushing::ChLoadNodeXYZBodyBushing(std::shared_ptr<ChNodeXYZ> nodeA, std::shared_ptr<ChBody> bodyB)
    : ChLoadNodeXYZBody(nodeA, bodyB), R(VNULL), is_stiff(false) {
    force_dX = chrono_types::make_shared<ChFunctionConst>(0.0);
    force_dY = chrono_types::make_shared<ChFunctionConst>(0.0);
    force_dZ = chrono_types::make_shared<ChFunctionConst>(0.0);
}

// Compute the force on the node, in absolute coordsystem, given position of node as abs_pos.
void ChLoadNodeXYZBodyBushing::ComputeForce(const ChFrameMoving<>& rel_AB, ChVector3d& loc_force) {
    loc_force = ChVector3d(force_dX->GetVal(rel_AB.GetPos().x()) - R.x() * rel_AB.GetPosDt().x(),
                           force_dY->GetVal(rel_AB.GetPos().y()) - R.y() * rel_AB.GetPosDt().y(),
                           force_dZ->GetVal(rel_AB.GetPos().z()) - R.z() * rel_AB.GetPosDt().z());
}

}  // end namespace chrono
