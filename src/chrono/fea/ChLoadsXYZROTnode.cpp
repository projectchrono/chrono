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

#include "chrono/fea/ChLoadsXYZROTnode.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
// ChLoadXYZROTnodeForce
// -----------------------------------------------------------------------------

ChLoadXYZROTnode::ChLoadXYZROTnode(std::shared_ptr<ChNodeFEAxyzrot> body) : ChLoadCustom(body) {
    computed_abs_force = VNULL;
    computed_abs_torque = VNULL;
}

void ChLoadXYZROTnode::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadable);
    if (!mnode->Variables().IsActive())
        return;

    ChFrameMoving<> bodycoordA;

    if (state_x) {
        // the numerical jacobian algo might change state_x
        bodycoordA.SetCoord(state_x->segment(0, 7));
    } else {
        bodycoordA.SetCoord(mnode->coord);
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        bodycoordA.SetPos_dt(state_w->segment(0, 3));
        bodycoordA.SetWvel_loc(state_w->segment(3, 3));
    } else {
        bodycoordA.SetCoord_dt(mnode->GetCoord_dt());
    }

    ComputeForceTorque(bodycoordA, computed_abs_force, computed_abs_torque);

    // Compute Q
    load_Q.segment(0, 3) = computed_abs_force.eigen();
    load_Q.segment(3, 3) =
        (bodycoordA.coord.rot.RotateBack(computed_abs_torque)).eigen();  // because Q expect torque in local frame
}

void ChLoadXYZROTnode::Update(double time) {
    ChLoadCustom::Update(time);
}

// -----------------------------------------------------------------------------
// ChLoadXYZnodeForceAbsolute
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeForceAbsolute::ChLoadXYZROTnodeForceAbsolute(std::shared_ptr<ChNodeFEAxyzrot> body,
                                                             const ChVector<>& force)
    : ChLoadXYZROTnode(body), m_force_base(force), m_scale(1) {
    m_modulation = chrono_types::make_shared<ChFunction_Const>(1.0);
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZROTnodeForceAbsolute::ComputeForceTorque(const ChFrameMoving<>& node_frame_abs_pos_vel,
                                                       ChVector<>& abs_force,
                                                       ChVector<>& abs_torque) {
    abs_force = GetForce();
}

void ChLoadXYZROTnodeForceAbsolute::Update(double time) {
    m_modulation->Update(time);
    m_scale = m_modulation->Get_y(time);
    ChLoadXYZROTnode::Update(time);
}

void ChLoadXYZROTnodeForceAbsolute::SetForceBase(const ChVector<>& force) {
    m_force_base = force;
}

ChVector<> ChLoadXYZROTnodeForceAbsolute::GetForce() const {
    return m_force_base * m_scale;
}

// -----------------------------------------------------------------------------
// ChLoadXYZROTnodeXYZROTnode
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeXYZROTnode::ChLoadXYZROTnodeXYZROTnode(std::shared_ptr<ChNodeFEAxyzrot> mbodyA,
                                                       std::shared_ptr<ChNodeFEAxyzrot> mbodyB,
                                                       const ChFrame<>& abs_application)
    : ChLoadCustomMultiple(mbodyA, mbodyB) {
    mbodyA->ChFrame::TransformParentToLocal(abs_application, loc_application_A);
    mbodyB->ChFrame::TransformParentToLocal(abs_application, loc_application_B);
}

void ChLoadXYZROTnodeXYZROTnode::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mbodyA = std::dynamic_pointer_cast<ChBody>(this->loadables[0]);
    auto mbodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);

    ChFrameMoving<> bodycoordA, bodycoordB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        bodycoordA.SetCoord(state_x->segment(0, 7));
        bodycoordB.SetCoord(state_x->segment(7, 7));
    } else {
        bodycoordA.SetCoord(mbodyA->coord);
        bodycoordB.SetCoord(mbodyB->coord);
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        bodycoordA.SetPos_dt(state_w->segment(0, 3));
        bodycoordA.SetWvel_loc(state_w->segment(3, 3));
        bodycoordB.SetPos_dt(state_w->segment(6, 3));
        bodycoordB.SetWvel_loc(state_w->segment(9, 3));
    } else {
        bodycoordA.SetCoord_dt(mbodyA->GetCoord_dt());
        bodycoordB.SetCoord_dt(mbodyB->GetCoord_dt());
    }

    frame_Aw = ChFrameMoving<>(loc_application_A) >> bodycoordA;
    frame_Bw = ChFrameMoving<>(loc_application_B) >> bodycoordB;
    ChFrameMoving<> rel_AB = frame_Aw >> frame_Bw.GetInverse();

    // COMPUTE THE FORCE

    ComputeForceTorque(rel_AB, locB_force, locB_torque);

    ChVector<> abs_force = frame_Bw.TransformDirectionLocalToParent(locB_force);
    ChVector<> abs_torque = frame_Bw.TransformDirectionLocalToParent(locB_torque);

    // Compute Q

    ChVector<> loc_ftorque = bodycoordA.GetRot().RotateBack(((frame_Aw.GetPos() - bodycoordA.GetPos()) % -abs_force));
    ChVector<> loc_torque = bodycoordA.GetRot().RotateBack(-abs_torque);
    load_Q.segment(0, 3) = -abs_force.eigen();
    load_Q.segment(3, 3) = (loc_ftorque + loc_torque).eigen();

    loc_ftorque = bodycoordB.GetRot().RotateBack(((frame_Bw.GetPos() - bodycoordB.GetPos()) % abs_force));
    loc_torque = bodycoordB.GetRot().RotateBack(abs_torque);
    load_Q.segment(6, 3) = abs_force.eigen();
    load_Q.segment(9, 3) = (loc_ftorque + loc_torque).eigen();
}

std::shared_ptr<ChNodeFEAxyzrot> ChLoadXYZROTnodeXYZROTnode::GetNodeA() const {
    return std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadables[0]);
}

std::shared_ptr<ChNodeFEAxyzrot> ChLoadXYZROTnodeXYZROTnode::GetNodeB() const {
    return std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadables[1]);
}

// -----------------------------------------------------------------------------
// ChLoadXYZROTnodeXYZROTnodeBushingSpherical
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeXYZROTnodeBushingSpherical::ChLoadXYZROTnodeXYZROTnodeBushingSpherical(
    std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
    std::shared_ptr<ChNodeFEAxyzrot> mnodeB,  ///< node B
    const ChFrame<>& abs_application,  ///< bushing location, in abs. coordinates. Will define loc_application_A and
                                       ///< loc_application_B
    const ChVector<>& mstiffness,      ///< stiffness, along x y z axes of the abs_application
    const ChVector<>& mdamping         ///< damping, along x y z axes of the abs_application
    )
    : ChLoadXYZROTnodeXYZROTnode(mnodeA, mnodeB, abs_application), stiffness(mstiffness), damping(mdamping) {}

void ChLoadXYZROTnodeXYZROTnodeBushingSpherical::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                                    ChVector<>& loc_force,
                                                                    ChVector<>& loc_torque) {
    loc_force = rel_AB.GetPos() * stiffness      // element-wise product!
                + rel_AB.GetPos_dt() * damping;  // element-wise product!
    loc_torque = VNULL;
}

// -----------------------------------------------------------------------------
// ChLoadXYZROTnodeXYZROTnodeBushingPlastic
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeXYZROTnodeBushingPlastic::ChLoadXYZROTnodeXYZROTnodeBushingPlastic(
    std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
    std::shared_ptr<ChNodeFEAxyzrot> mnodeB,  ///< node B
    const ChFrame<>& abs_application,
    const ChVector<>& mstiffness,
    const ChVector<>& mdamping,
    const ChVector<>& myield)
    : ChLoadXYZROTnodeXYZROTnodeBushingSpherical(mnodeA, mnodeB, abs_application, mstiffness, mdamping),
      yield(myield),
      plastic_def(VNULL) {}

void ChLoadXYZROTnodeXYZROTnodeBushingPlastic::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                                  ChVector<>& loc_force,
                                                                  ChVector<>& loc_torque) {
    loc_force = (rel_AB.GetPos() - plastic_def) * stiffness  // element-wise product!
                + rel_AB.GetPos_dt() * damping;              // element-wise product!

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

    // GetLog() << "loc_force" << loc_force << "\n";
    // GetLog() << "plastic_def" << plastic_def << "\n";
    loc_torque = VNULL;
}

// -----------------------------------------------------------------------------
// ChLoadBodyBodyBushingMate
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeXYZROTnodeBushingMate::ChLoadXYZROTnodeXYZROTnodeBushingMate(
    std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
    std::shared_ptr<ChNodeFEAxyzrot> mnodeB,  ///< node B
    const ChFrame<>& abs_application,
    const ChVector<>& mstiffness,
    const ChVector<>& mdamping,
    const ChVector<>& mrotstiffness,
    const ChVector<>& mrotdamping)
    : ChLoadXYZROTnodeXYZROTnodeBushingSpherical(mnodeA, mnodeB, abs_application, mstiffness, mdamping),
      rot_stiffness(mrotstiffness),
      rot_damping(mrotdamping) {}

void ChLoadXYZROTnodeXYZROTnodeBushingMate::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                               ChVector<>& loc_force,
                                                               ChVector<>& loc_torque) {
    // inherit parent to compute loc_force = ...
    ChLoadXYZROTnodeXYZROTnodeBushingSpherical::ComputeForceTorque(rel_AB, loc_force, loc_torque);

    // compute local torque using small rotations:
    ChQuaternion<> rel_rot = rel_AB.GetRot();

    ChVector<> dir_rot;
    double angle_rot;
    rel_rot.Q_to_AngAxis(angle_rot, dir_rot);
    if (angle_rot > CH_C_PI)
        angle_rot -= CH_C_2PI;
    if (angle_rot < -CH_C_PI)
        angle_rot += CH_C_2PI;
    ChVector<> vect_rot = dir_rot * angle_rot;

    loc_torque = vect_rot * rot_stiffness               // element-wise product!
                 + rel_AB.GetWvel_par() * rot_damping;  // element-wise product!
}

// -----------------------------------------------------------------------------
// ChLoadBodyBodyBushingGeneric
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeXYZROTnodeBushingGeneric::ChLoadXYZROTnodeXYZROTnodeBushingGeneric(
    std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
    std::shared_ptr<ChNodeFEAxyzrot> mnodeB,  ///< node B
    const ChFrame<>& abs_application,
    ChMatrixConstRef mstiffness,
    ChMatrixConstRef mdamping)
    : ChLoadXYZROTnodeXYZROTnode(mnodeA, mnodeB, abs_application), stiffness(mstiffness), damping(mdamping) {}

void ChLoadXYZROTnodeXYZROTnodeBushingGeneric::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                                  ChVector<>& loc_force,
                                                                  ChVector<>& loc_torque) {
    // compute local force & torque (assuming small rotations):
    ChVectorDynamic<> mF(6);
    ChVectorDynamic<> mS(6);
    ChVectorDynamic<> mSdt(6);
    ChVector<> rel_pos = rel_AB.GetPos() + neutral_displacement.GetPos();
    ChQuaternion<> rel_rot = rel_AB.GetRot() * neutral_displacement.GetRot();
    ChVector<> dir_rot;
    double angle_rot;
    rel_rot.Q_to_AngAxis(angle_rot, dir_rot);
    if (angle_rot > CH_C_PI)
        angle_rot -= CH_C_2PI;
    if (angle_rot < -CH_C_PI)
        angle_rot += CH_C_2PI;
    ChVector<> vect_rot = dir_rot * angle_rot;

    mS.segment(0, 3) = rel_pos.eigen();
    mS.segment(3, 3) = vect_rot.eigen();
    mSdt.segment(0, 3) = rel_AB.GetPos_dt().eigen();
    mSdt.segment(3, 3) = rel_AB.GetWvel_par().eigen();

    mF = stiffness * mS + damping * mSdt;

    loc_force = ChVector<>(mF.segment(0, 3)) - neutral_force;
    loc_torque = ChVector<>(mF.segment(3, 3)) - neutral_torque;
}

// -----------------------------------------------------------------------------
// ChLoadXYZROTnodeBody
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeBody::ChLoadXYZROTnodeBody(std::shared_ptr<ChNodeFEAxyzrot> mnodeA,
                                           std::shared_ptr<ChBody> mbodyB,
                                           const ChFrame<>& abs_application)
    : ChLoadCustomMultiple(mnodeA, mbodyB) {
    mnodeA->ChFrame::TransformParentToLocal(abs_application, loc_application_A);
    mbodyB->ChFrame::TransformParentToLocal(abs_application, loc_application_B);
}

void ChLoadXYZROTnodeBody::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mbodyA = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadables[0]);
    auto mbodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);

    ChFrameMoving<> bodycoordA, bodycoordB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        bodycoordA.SetCoord(state_x->segment(0, 7));
        bodycoordB.SetCoord(state_x->segment(7, 7));
    } else {
        bodycoordA.SetCoord(mbodyA->coord);
        bodycoordB.SetCoord(mbodyB->coord);
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        bodycoordA.SetPos_dt(state_w->segment(0, 3));
        bodycoordA.SetWvel_loc(state_w->segment(3, 3));
        bodycoordB.SetPos_dt(state_w->segment(6, 3));
        bodycoordB.SetWvel_loc(state_w->segment(9, 3));
    } else {
        bodycoordA.SetCoord_dt(mbodyA->GetCoord_dt());
        bodycoordB.SetCoord_dt(mbodyB->GetCoord_dt());
    }

    frame_Aw = ChFrameMoving<>(loc_application_A) >> bodycoordA;
    frame_Bw = ChFrameMoving<>(loc_application_B) >> bodycoordB;
    ChFrameMoving<> rel_AB = frame_Aw >> frame_Bw.GetInverse();

    // COMPUTE THE FORCE

    ComputeForceTorque(rel_AB, locB_force, locB_torque);

    ChVector<> abs_force = frame_Bw.TransformDirectionLocalToParent(locB_force);
    ChVector<> abs_torque = frame_Bw.TransformDirectionLocalToParent(locB_torque);

    // Compute Q

    ChVector<> loc_ftorque = bodycoordA.GetRot().RotateBack(((frame_Aw.GetPos() - bodycoordA.GetPos()) % -abs_force));
    ChVector<> loc_torque = bodycoordA.GetRot().RotateBack(-abs_torque);
    load_Q.segment(0, 3) = -abs_force.eigen();
    load_Q.segment(3, 3) = (loc_ftorque + loc_torque).eigen();

    loc_ftorque = bodycoordB.GetRot().RotateBack(((frame_Bw.GetPos() - bodycoordB.GetPos()) % abs_force));
    loc_torque = bodycoordB.GetRot().RotateBack(abs_torque);
    load_Q.segment(6, 3) = abs_force.eigen();
    load_Q.segment(9, 3) = (loc_ftorque + loc_torque).eigen();
}

std::shared_ptr<ChNodeFEAxyzrot> ChLoadXYZROTnodeBody::GetNodeA() const {
    return std::dynamic_pointer_cast<ChNodeFEAxyzrot>(this->loadables[0]);
}

std::shared_ptr<ChBody> ChLoadXYZROTnodeBody::GetBodyB() const {
    return std::dynamic_pointer_cast<ChBody>(this->loadables[1]);
}

// -----------------------------------------------------------------------------
// ChLoadXYZROTnodeBodyBushingSpherical
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeBodyBushingSpherical::ChLoadXYZROTnodeBodyBushingSpherical(std::shared_ptr<ChNodeFEAxyzrot> mnodeA,
                                                                           std::shared_ptr<ChBody> mbodyB,
                                                                           const ChFrame<>& abs_application,
                                                                           const ChVector<>& mstiffness,
                                                                           const ChVector<>& mdamping)
    : ChLoadXYZROTnodeBody(mnodeA, mbodyB, abs_application), stiffness(mstiffness), damping(mdamping) {}

void ChLoadXYZROTnodeBodyBushingSpherical::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                              ChVector<>& loc_force,
                                                              ChVector<>& loc_torque) {
    loc_force = rel_AB.GetPos() * stiffness      // element-wise product!
                + rel_AB.GetPos_dt() * damping;  // element-wise product!
    loc_torque = VNULL;
}

// -----------------------------------------------------------------------------
// ChLoadXYZROTnodeBodyBushingPlastic
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeBodyBushingPlastic::ChLoadXYZROTnodeBodyBushingPlastic(std::shared_ptr<ChNodeFEAxyzrot> mnodeA,
                                                                       std::shared_ptr<ChBody> mbodyB,
                                                                       const ChFrame<>& abs_application,
                                                                       const ChVector<>& mstiffness,
                                                                       const ChVector<>& mdamping,
                                                                       const ChVector<>& myield)
    : ChLoadXYZROTnodeBodyBushingSpherical(mnodeA, mbodyB, abs_application, mstiffness, mdamping),
      yield(myield),
      plastic_def(VNULL) {}

void ChLoadXYZROTnodeBodyBushingPlastic::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                            ChVector<>& loc_force,
                                                            ChVector<>& loc_torque) {
    loc_force = (rel_AB.GetPos() - plastic_def) * stiffness  // element-wise product!
                + rel_AB.GetPos_dt() * damping;              // element-wise product!

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

    // GetLog() << "loc_force" << loc_force << "\n";
    // GetLog() << "plastic_def" << plastic_def << "\n";
    loc_torque = VNULL;
}

// -----------------------------------------------------------------------------
// ChLoadXYZROTnodeBodyBushingMate
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeBodyBushingMate::ChLoadXYZROTnodeBodyBushingMate(std::shared_ptr<ChNodeFEAxyzrot> mnodeA,
                                                                 std::shared_ptr<ChBody> mbodyB,
                                                                 const ChFrame<>& abs_application,
                                                                 const ChVector<>& mstiffness,
                                                                 const ChVector<>& mdamping,
                                                                 const ChVector<>& mrotstiffness,
                                                                 const ChVector<>& mrotdamping)
    : ChLoadXYZROTnodeBodyBushingSpherical(mnodeA, mbodyB, abs_application, mstiffness, mdamping),
      rot_stiffness(mrotstiffness),
      rot_damping(mrotdamping) {}

void ChLoadXYZROTnodeBodyBushingMate::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                         ChVector<>& loc_force,
                                                         ChVector<>& loc_torque) {
    // inherit parent to compute loc_force = ...
    ChLoadXYZROTnodeBodyBushingSpherical::ComputeForceTorque(rel_AB, loc_force, loc_torque);

    // compute local torque using small rotations:
    ChQuaternion<> rel_rot = rel_AB.GetRot();

    ChVector<> dir_rot;
    double angle_rot;
    rel_rot.Q_to_AngAxis(angle_rot, dir_rot);
    if (angle_rot > CH_C_PI)
        angle_rot -= CH_C_2PI;
    if (angle_rot < -CH_C_PI)
        angle_rot += CH_C_2PI;
    ChVector<> vect_rot = dir_rot * angle_rot;

    loc_torque = vect_rot * rot_stiffness               // element-wise product!
                 + rel_AB.GetWvel_par() * rot_damping;  // element-wise product!
}

// -----------------------------------------------------------------------------
// ChLoadXYZROTnodeBodyBushingGeneric
// -----------------------------------------------------------------------------

ChLoadXYZROTnodeBodyBushingGeneric::ChLoadXYZROTnodeBodyBushingGeneric(std::shared_ptr<ChNodeFEAxyzrot> mnodeA,
                                                                       std::shared_ptr<ChBody> mbodyB,
                                                                       const ChFrame<>& abs_application,
                                                                       ChMatrixConstRef mstiffness,
                                                                       ChMatrixConstRef mdamping)
    : ChLoadXYZROTnodeBody(mnodeA, mbodyB, abs_application), stiffness(mstiffness), damping(mdamping) {}

void ChLoadXYZROTnodeBodyBushingGeneric::ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                                            ChVector<>& loc_force,
                                                            ChVector<>& loc_torque) {
    // compute local force & torque (assuming small rotations):
    ChVectorDynamic<> mF(6);
    ChVectorDynamic<> mS(6);
    ChVectorDynamic<> mSdt(6);
    ChVector<> rel_pos = rel_AB.GetPos() + neutral_displacement.GetPos();
    ChQuaternion<> rel_rot = rel_AB.GetRot() * neutral_displacement.GetRot();
    ChVector<> dir_rot;
    double angle_rot;
    rel_rot.Q_to_AngAxis(angle_rot, dir_rot);
    if (angle_rot > CH_C_PI)
        angle_rot -= CH_C_2PI;
    if (angle_rot < -CH_C_PI)
        angle_rot += CH_C_2PI;
    ChVector<> vect_rot = dir_rot * angle_rot;

    mS.segment(0, 3) = rel_pos.eigen();
    mS.segment(3, 3) = vect_rot.eigen();
    mSdt.segment(0, 3) = rel_AB.GetPos_dt().eigen();
    mSdt.segment(3, 3) = rel_AB.GetWvel_par().eigen();

    mF = stiffness * mS + damping * mSdt;

    loc_force = ChVector<>(mF.segment(0, 3)) - neutral_force;
    loc_torque = ChVector<>(mF.segment(3, 3)) - neutral_torque;
}

}  // namespace fea
}  // namespace chrono
