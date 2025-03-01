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

#include "chrono/fea/ChLoadsNodeXYZRot.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRot
// -----------------------------------------------------------------------------

ChLoadNodeXYZRot::ChLoadNodeXYZRot(std::shared_ptr<ChNodeFEAxyzrot> node) : ChLoadCustom(node) {
    computed_abs_force = VNULL;
    computed_abs_torque = VNULL;
}

void ChLoadNodeXYZRot::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadable);
    if (!mnode->Variables().IsActive())
        return;

    ChFrameMoving<> bodycoordA;

    if (state_x) {
        // the numerical jacobian algo might change state_x
        bodycoordA.SetCoordsys(state_x->segment(0, 7));
    } else {
        bodycoordA.SetCoordsys(mnode->GetCoordsys());
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        bodycoordA.SetPosDt(state_w->segment(0, 3));
        bodycoordA.SetAngVelLocal(state_w->segment(3, 3));
    } else {
        bodycoordA.SetCoordsysDt(mnode->GetCoordsysDt());
    }

    ComputeForceTorque(bodycoordA, computed_abs_force, computed_abs_torque);

    // Compute Q
    load_Q.segment(0, 3) = computed_abs_force.eigen();
    load_Q.segment(3, 3) =
        (bodycoordA.GetRot().RotateBack(computed_abs_torque)).eigen();  // because Q expect torque in local frame
}

void ChLoadNodeXYZRot::Update(double time, bool update_assets) {
    ChLoadCustom::Update(time, update_assets);
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotForceAbs
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotForceAbs::ChLoadNodeXYZRotForceAbs(std::shared_ptr<ChNodeFEAxyzrot> body, const ChVector3d& force)
    : ChLoadNodeXYZRot(body), m_force_base(force), m_scale(1) {
    m_modulation = chrono_types::make_shared<ChFunctionConst>(1.0);
}

// Compute the force on the node, in absolute coordsystem, given position of node as abs_pos.
void ChLoadNodeXYZRotForceAbs::ComputeForceTorque(const ChFrameMoving<>& node_frame_abs_pos_vel,
                                                  ChVector3d& abs_force,
                                                  ChVector3d& abs_torque) {
    abs_force = GetForce();
}

void ChLoadNodeXYZRotForceAbs::Update(double time, bool update_assets) {
    m_modulation->Update(time);
    m_scale = m_modulation->GetVal(time);
    ChLoadNodeXYZRot::Update(time, update_assets);
}

void ChLoadNodeXYZRotForceAbs::SetForceBase(const ChVector3d& force) {
    m_force_base = force;
}

ChVector3d ChLoadNodeXYZRotForceAbs::GetForce() const {
    return m_force_base * m_scale;
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotNodeXYZRot
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotNodeXYZRot::ChLoadNodeXYZRotNodeXYZRot(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                                                       std::shared_ptr<ChNodeFEAxyzrot> nodeB,
                                                       const ChFrame<>& abs_application)
    : ChLoadCustomMultiple(nodeA, nodeB) {
    loc_application_A = nodeA->ChFrame::TransformParentToLocal(abs_application);
    loc_application_B = nodeB->ChFrame::TransformParentToLocal(abs_application);
}

void ChLoadNodeXYZRotNodeXYZRot::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mbodyA = std::dynamic_pointer_cast<ChBody>(this->loadables[0]);
    auto mbodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);

    ChFrameMoving<> bodycoordA, bodycoordB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        bodycoordA.SetCoordsys(state_x->segment(0, 7));
        bodycoordB.SetCoordsys(state_x->segment(7, 7));
    } else {
        bodycoordA.SetCoordsys(mbodyA->GetCoordsys());
        bodycoordB.SetCoordsys(mbodyB->GetCoordsys());
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        bodycoordA.SetPosDt(state_w->segment(0, 3));
        bodycoordA.SetAngVelLocal(state_w->segment(3, 3));
        bodycoordB.SetPosDt(state_w->segment(6, 3));
        bodycoordB.SetAngVelLocal(state_w->segment(9, 3));
    } else {
        bodycoordA.SetCoordsysDt(mbodyA->GetCoordsysDt());
        bodycoordB.SetCoordsysDt(mbodyB->GetCoordsysDt());
    }

    frame_Aw = ChFrameMoving<>(loc_application_A) >> bodycoordA;
    frame_Bw = ChFrameMoving<>(loc_application_B) >> bodycoordB;
    ChFrameMoving<> rel_AB = frame_Aw >> frame_Bw.GetInverse();

    // COMPUTE THE FORCE

    ComputeForceTorque(rel_AB, locB_force, locB_torque);

    ChVector3d abs_force = frame_Bw.TransformDirectionLocalToParent(locB_force);
    ChVector3d abs_torque = frame_Bw.TransformDirectionLocalToParent(locB_torque);

    // Compute Q

    ChVector3d loc_ftorque = bodycoordA.GetRot().RotateBack(((frame_Aw.GetPos() - bodycoordA.GetPos()) % -abs_force));
    ChVector3d loc_torque = bodycoordA.GetRot().RotateBack(-abs_torque);
    load_Q.segment(0, 3) = -abs_force.eigen();
    load_Q.segment(3, 3) = (loc_ftorque + loc_torque).eigen();

    loc_ftorque = bodycoordB.GetRot().RotateBack(((frame_Bw.GetPos() - bodycoordB.GetPos()) % abs_force));
    loc_torque = bodycoordB.GetRot().RotateBack(abs_torque);
    load_Q.segment(6, 3) = abs_force.eigen();
    load_Q.segment(9, 3) = (loc_ftorque + loc_torque).eigen();
}

std::shared_ptr<ChNodeFEAxyzrot> ChLoadNodeXYZRotNodeXYZRot::GetNodeA() const {
    return std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadables[0]);
}

std::shared_ptr<ChNodeFEAxyzrot> ChLoadNodeXYZRotNodeXYZRot::GetNodeB() const {
    return std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadables[1]);
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotNodeXYZRotBushingSpherical
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotNodeXYZRotBushingSpherical::ChLoadNodeXYZRotNodeXYZRotBushingSpherical(
    std::shared_ptr<ChNodeFEAxyzrot> nodeA,
    std::shared_ptr<ChNodeFEAxyzrot> nodeB,
    const ChFrame<>& abs_application,
    const ChVector3d& stiffness_coefs,
    const ChVector3d& damping_coefs)
    : ChLoadNodeXYZRotNodeXYZRot(nodeA, nodeB, abs_application), stiffness(stiffness_coefs), damping(damping_coefs) {}

void ChLoadNodeXYZRotNodeXYZRotBushingSpherical::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                                    ChVector3d& loc_force,
                                                                    ChVector3d& loc_torque) {
    loc_force = rel_AB.GetPos() * stiffness     // element-wise product!
                + rel_AB.GetPosDt() * damping;  // element-wise product!
    loc_torque = VNULL;
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotNodeXYZRotBushingPlastic
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotNodeXYZRotBushingPlastic::ChLoadNodeXYZRotNodeXYZRotBushingPlastic(
    std::shared_ptr<ChNodeFEAxyzrot> nodeA,
    std::shared_ptr<ChNodeFEAxyzrot> nodeB,
    const ChFrame<>& abs_application,
    const ChVector3d& stiffness_coefs,
    const ChVector3d& damping_coefs,
    const ChVector3d& yield_coefs)
    : ChLoadNodeXYZRotNodeXYZRotBushingSpherical(nodeA, nodeB, abs_application, stiffness_coefs, damping_coefs),
      yield(yield_coefs),
      plastic_def(VNULL) {}

void ChLoadNodeXYZRotNodeXYZRotBushingPlastic::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                                  ChVector3d& loc_force,
                                                                  ChVector3d& loc_torque) {
    loc_force = (rel_AB.GetPos() - plastic_def) * stiffness  // element-wise product!
                + rel_AB.GetPosDt() * damping;               // element-wise product!

    // A basic plasticity, assumed with box capping, without hardening:

    if (loc_force.x() > yield.x()) {
        loc_force.x() = yield.x();
        plastic_def.x() = rel_AB.GetPos().x() - loc_force.x() / stiffness.x();
    }
    if (loc_force.x() < -yield.x()) {
        loc_force.x() = -yield.x();
        plastic_def.x() = rel_AB.GetPos().x() - loc_force.x() / stiffness.x();
    }
    if (loc_force.y() > yield.y()) {
        loc_force.y() = yield.y();
        plastic_def.y() = rel_AB.GetPos().y() - loc_force.y() / stiffness.y();
    }
    if (loc_force.y() < -yield.y()) {
        loc_force.y() = -yield.y();
        plastic_def.y() = rel_AB.GetPos().y() - loc_force.y() / stiffness.y();
    }
    if (loc_force.z() > yield.z()) {
        loc_force.z() = yield.z();
        plastic_def.z() = rel_AB.GetPos().z() - loc_force.z() / stiffness.z();
    }
    if (loc_force.z() < -yield.z()) {
        loc_force.z() = -yield.z();
        plastic_def.z() = rel_AB.GetPos().z() - loc_force.z() / stiffness.z();
    }

    // std::cout << "loc_force" << loc_force << std::endl;
    // std::cout << "plastic_def" << plastic_def << std::endl;
    loc_torque = VNULL;
}

// -----------------------------------------------------------------------------
// ChLoadBodyBodyBushingMate
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotNodeXYZRotBushingMate::ChLoadNodeXYZRotNodeXYZRotBushingMate(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                                                                             std::shared_ptr<ChNodeFEAxyzrot> nodeB,
                                                                             const ChFrame<>& abs_application,
                                                                             const ChVector3d& stiffness_coefs,
                                                                             const ChVector3d& damping_coefs,
                                                                             const ChVector3d& rotstiffness_coefs,
                                                                             const ChVector3d& rotdamping_coefs)
    : ChLoadNodeXYZRotNodeXYZRotBushingSpherical(nodeA, nodeB, abs_application, stiffness_coefs, damping_coefs),
      rot_stiffness(rotstiffness_coefs),
      rot_damping(rotdamping_coefs) {}

void ChLoadNodeXYZRotNodeXYZRotBushingMate::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                               ChVector3d& loc_force,
                                                               ChVector3d& loc_torque) {
    // inherit parent to compute loc_force = ...
    ChLoadNodeXYZRotNodeXYZRotBushingSpherical::ComputeForceTorque(rel_AB, loc_force, loc_torque);

    // compute local torque using small rotations:
    ChQuaternion<> rel_rot = rel_AB.GetRot();

    ChVector3d dir_rot;
    double angle_rot;
    rel_rot.GetAngleAxis(angle_rot, dir_rot);
    if (angle_rot > CH_PI)
        angle_rot -= CH_2PI;
    if (angle_rot < -CH_PI)
        angle_rot += CH_2PI;
    ChVector3d vect_rot = dir_rot * angle_rot;

    loc_torque = vect_rot * rot_stiffness                   // element-wise product!
                 + rel_AB.GetAngVelParent() * rot_damping;  // element-wise product!
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotNodeXYZRotBushingGeneric
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotNodeXYZRotBushingGeneric::ChLoadNodeXYZRotNodeXYZRotBushingGeneric(
    std::shared_ptr<ChNodeFEAxyzrot> nodeA,
    std::shared_ptr<ChNodeFEAxyzrot> nodeB,
    const ChFrame<>& abs_application,
    ChMatrixConstRef stiffness_coefs,
    ChMatrixConstRef damping_coefs)
    : ChLoadNodeXYZRotNodeXYZRot(nodeA, nodeB, abs_application), stiffness(stiffness_coefs), damping(damping_coefs) {}

void ChLoadNodeXYZRotNodeXYZRotBushingGeneric::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                                  ChVector3d& loc_force,
                                                                  ChVector3d& loc_torque) {
    // compute local force & torque (assuming small rotations):
    ChVectorDynamic<> mF(6);
    ChVectorDynamic<> mS(6);
    ChVectorDynamic<> mSdt(6);
    ChVector3d rel_pos = rel_AB.GetPos() + neutral_displacement.GetPos();
    ChQuaternion<> rel_rot = rel_AB.GetRot() * neutral_displacement.GetRot();
    ChVector3d dir_rot;
    double angle_rot;
    rel_rot.GetAngleAxis(angle_rot, dir_rot);
    if (angle_rot > CH_PI)
        angle_rot -= CH_2PI;
    if (angle_rot < -CH_PI)
        angle_rot += CH_2PI;
    ChVector3d vect_rot = dir_rot * angle_rot;

    mS.segment(0, 3) = rel_pos.eigen();
    mS.segment(3, 3) = vect_rot.eigen();
    mSdt.segment(0, 3) = rel_AB.GetPosDt().eigen();
    mSdt.segment(3, 3) = rel_AB.GetAngVelParent().eigen();

    mF = stiffness * mS + damping * mSdt;

    loc_force = ChVector3d(mF.segment(0, 3)) - neutral_force;
    loc_torque = ChVector3d(mF.segment(3, 3)) - neutral_torque;
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotBody
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotBody::ChLoadNodeXYZRotBody(std::shared_ptr<ChNodeFEAxyzrot> node,
                                           std::shared_ptr<ChBody> body,
                                           const ChFrame<>& abs_application)
    : ChLoadCustomMultiple(node, body) {
    loc_application_A = node->ChFrame::TransformParentToLocal(abs_application);
    loc_application_B = body->ChFrame::TransformParentToLocal(abs_application);
}

void ChLoadNodeXYZRotBody::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto bodyA = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadables[0]);
    auto bodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);

    ChFrameMoving<> bodycoordA, bodycoordB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        bodycoordA.SetCoordsys(state_x->segment(0, 7));
        bodycoordB.SetCoordsys(state_x->segment(7, 7));
    } else {
        bodycoordA.SetCoordsys(bodyA->GetCoordsys());
        bodycoordB.SetCoordsys(bodyB->GetCoordsys());
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        bodycoordA.SetPosDt(state_w->segment(0, 3));
        bodycoordA.SetAngVelLocal(state_w->segment(3, 3));
        bodycoordB.SetPosDt(state_w->segment(6, 3));
        bodycoordB.SetAngVelLocal(state_w->segment(9, 3));
    } else {
        bodycoordA.SetCoordsysDt(bodyA->GetCoordsysDt());
        bodycoordB.SetCoordsysDt(bodyB->GetCoordsysDt());
    }

    frame_Aw = ChFrameMoving<>(loc_application_A) >> bodycoordA;
    frame_Bw = ChFrameMoving<>(loc_application_B) >> bodycoordB;
    ChFrameMoving<> rel_AB = frame_Aw >> frame_Bw.GetInverse();

    // COMPUTE THE FORCE

    ComputeForceTorque(rel_AB, locB_force, locB_torque);

    ChVector3d abs_force = frame_Bw.TransformDirectionLocalToParent(locB_force);
    ChVector3d abs_torque = frame_Bw.TransformDirectionLocalToParent(locB_torque);

    // Compute Q

    ChVector3d loc_ftorque = bodycoordA.GetRot().RotateBack(((frame_Aw.GetPos() - bodycoordA.GetPos()) % -abs_force));
    ChVector3d loc_torque = bodycoordA.GetRot().RotateBack(-abs_torque);
    load_Q.segment(0, 3) = -abs_force.eigen();
    load_Q.segment(3, 3) = (loc_ftorque + loc_torque).eigen();

    loc_ftorque = bodycoordB.GetRot().RotateBack(((frame_Bw.GetPos() - bodycoordB.GetPos()) % abs_force));
    loc_torque = bodycoordB.GetRot().RotateBack(abs_torque);
    load_Q.segment(6, 3) = abs_force.eigen();
    load_Q.segment(9, 3) = (loc_ftorque + loc_torque).eigen();
}

std::shared_ptr<ChNodeFEAxyzrot> ChLoadNodeXYZRotBody::GetNode() const {
    return std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadables[0]);
}

std::shared_ptr<ChBody> ChLoadNodeXYZRotBody::GetBody() const {
    return std::dynamic_pointer_cast<ChBody>(this->loadables[1]);
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotBodyBushingSpherical
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotBodyBushingSpherical::ChLoadNodeXYZRotBodyBushingSpherical(std::shared_ptr<ChNodeFEAxyzrot> node,
                                                                           std::shared_ptr<ChBody> body,
                                                                           const ChFrame<>& abs_application,
                                                                           const ChVector3d& stiffness_coefs,
                                                                           const ChVector3d& damping_coefs)
    : ChLoadNodeXYZRotBody(node, body, abs_application), stiffness(stiffness_coefs), damping(damping_coefs) {}

void ChLoadNodeXYZRotBodyBushingSpherical::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                              ChVector3d& loc_force,
                                                              ChVector3d& loc_torque) {
    loc_force = rel_AB.GetPos() * stiffness     // element-wise product!
                + rel_AB.GetPosDt() * damping;  // element-wise product!
    loc_torque = VNULL;
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotBodyBushingPlastic
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotBodyBushingPlastic::ChLoadNodeXYZRotBodyBushingPlastic(std::shared_ptr<ChNodeFEAxyzrot> node,
                                                                       std::shared_ptr<ChBody> body,
                                                                       const ChFrame<>& abs_application,
                                                                       const ChVector3d& stiffness_coefs,
                                                                       const ChVector3d& damping_coefs,
                                                                       const ChVector3d& myield)
    : ChLoadNodeXYZRotBodyBushingSpherical(node, body, abs_application, stiffness_coefs, damping_coefs),
      yield(myield),
      plastic_def(VNULL) {}

void ChLoadNodeXYZRotBodyBushingPlastic::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                            ChVector3d& loc_force,
                                                            ChVector3d& loc_torque) {
    loc_force = (rel_AB.GetPos() - plastic_def) * stiffness  // element-wise product!
                + rel_AB.GetPosDt() * damping;               // element-wise product!

    // A basic plasticity, assumed with box capping, without hardening:

    if (loc_force.x() > yield.x()) {
        loc_force.x() = yield.x();
        plastic_def.x() = rel_AB.GetPos().x() - loc_force.x() / stiffness.x();
    }
    if (loc_force.x() < -yield.x()) {
        loc_force.x() = -yield.x();
        plastic_def.x() = rel_AB.GetPos().x() - loc_force.x() / stiffness.x();
    }
    if (loc_force.y() > yield.y()) {
        loc_force.y() = yield.y();
        plastic_def.y() = rel_AB.GetPos().y() - loc_force.y() / stiffness.y();
    }
    if (loc_force.y() < -yield.y()) {
        loc_force.y() = -yield.y();
        plastic_def.y() = rel_AB.GetPos().y() - loc_force.y() / stiffness.y();
    }
    if (loc_force.z() > yield.z()) {
        loc_force.z() = yield.z();
        plastic_def.z() = rel_AB.GetPos().z() - loc_force.z() / stiffness.z();
    }
    if (loc_force.z() < -yield.z()) {
        loc_force.z() = -yield.z();
        plastic_def.z() = rel_AB.GetPos().z() - loc_force.z() / stiffness.z();
    }

    // std::cout << "loc_force" << loc_force << std::endl;
    // std::cout << "plastic_def" << plastic_def << std::endl;
    loc_torque = VNULL;
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotBodyBushingMate
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotBodyBushingMate::ChLoadNodeXYZRotBodyBushingMate(std::shared_ptr<ChNodeFEAxyzrot> node,
                                                                 std::shared_ptr<ChBody> body,
                                                                 const ChFrame<>& abs_application,
                                                                 const ChVector3d& stiffness_coefs,
                                                                 const ChVector3d& damping_coefs,
                                                                 const ChVector3d& rotstiffness_coefs,
                                                                 const ChVector3d& rotdamping_coefs)
    : ChLoadNodeXYZRotBodyBushingSpherical(node, body, abs_application, stiffness_coefs, damping_coefs),
      rot_stiffness(rotstiffness_coefs),
      rot_damping(rotdamping_coefs) {}

void ChLoadNodeXYZRotBodyBushingMate::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                         ChVector3d& loc_force,
                                                         ChVector3d& loc_torque) {
    // inherit parent to compute loc_force = ...
    ChLoadNodeXYZRotBodyBushingSpherical::ComputeForceTorque(rel_AB, loc_force, loc_torque);

    // compute local torque using small rotations:
    ChQuaternion<> rel_rot = rel_AB.GetRot();

    ChVector3d dir_rot;
    double angle_rot;
    rel_rot.GetAngleAxis(angle_rot, dir_rot);
    if (angle_rot > CH_PI)
        angle_rot -= CH_2PI;
    if (angle_rot < -CH_PI)
        angle_rot += CH_2PI;
    ChVector3d vect_rot = dir_rot * angle_rot;

    loc_torque = vect_rot * rot_stiffness                   // element-wise product!
                 + rel_AB.GetAngVelParent() * rot_damping;  // element-wise product!
}

// -----------------------------------------------------------------------------
// ChLoadNodeXYZRotBodyBushingGeneric
// -----------------------------------------------------------------------------

ChLoadNodeXYZRotBodyBushingGeneric::ChLoadNodeXYZRotBodyBushingGeneric(std::shared_ptr<ChNodeFEAxyzrot> node,
                                                                       std::shared_ptr<ChBody> body,
                                                                       const ChFrame<>& abs_application,
                                                                       ChMatrixConstRef stiffness_coefs,
                                                                       ChMatrixConstRef damping_coefs)
    : ChLoadNodeXYZRotBody(node, body, abs_application), stiffness(stiffness_coefs), damping(damping_coefs) {}

void ChLoadNodeXYZRotBodyBushingGeneric::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                            ChVector3d& loc_force,
                                                            ChVector3d& loc_torque) {
    // compute local force & torque (assuming small rotations):
    ChVectorDynamic<> mF(6);
    ChVectorDynamic<> mS(6);
    ChVectorDynamic<> mSdt(6);
    ChVector3d rel_pos = rel_AB.GetPos() + neutral_displacement.GetPos();
    ChQuaternion<> rel_rot = rel_AB.GetRot() * neutral_displacement.GetRot();
    ChVector3d dir_rot;
    double angle_rot;
    rel_rot.GetAngleAxis(angle_rot, dir_rot);
    if (angle_rot > CH_PI)
        angle_rot -= CH_2PI;
    if (angle_rot < -CH_PI)
        angle_rot += CH_2PI;
    ChVector3d vect_rot = dir_rot * angle_rot;

    mS.segment(0, 3) = rel_pos.eigen();
    mS.segment(3, 3) = vect_rot.eigen();
    mSdt.segment(0, 3) = rel_AB.GetPosDt().eigen();
    mSdt.segment(3, 3) = rel_AB.GetAngVelParent().eigen();

    mF = stiffness * mS + damping * mSdt;

    loc_force = ChVector3d(mF.segment(0, 3)) - neutral_force;
    loc_torque = ChVector3d(mF.segment(3, 3)) - neutral_torque;
}

}  // namespace fea
}  // namespace chrono
