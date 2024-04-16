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

#include "chrono/physics/ChLoadsBody.h"

namespace chrono {

// -----------------------------------------------------------------------------
// ChLoadBodyForce
// -----------------------------------------------------------------------------

ChLoadBodyForce::ChLoadBodyForce(std::shared_ptr<ChBody> body,
                                 const ChVector3d& force,
                                 bool local_force,
                                 const ChVector3d& point,
                                 bool local_point)
    : ChLoadCustom(body),
      m_force(force),
      m_point(point),
      m_local_force(local_force),
      m_local_point(local_point),
      m_scale(1) {
    m_modulation = chrono_types::make_shared<ChFunctionConst>(1.0);
}

void ChLoadBodyForce::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mbody = std::dynamic_pointer_cast<ChBody>(this->loadable);
    if (!mbody->Variables().IsActive())
        return;

    ChVector3d abs_force;
    if (m_local_force)
        abs_force = mbody->TransformDirectionLocalToParent(m_force);
    else
        abs_force = m_force;

    abs_force *= m_scale;

    ChVector3d abs_point;
    if (m_local_point)
        abs_point = mbody->TransformPointLocalToParent(m_point);
    else
        abs_point = m_point;

    // ChBody assumes F={force_abs, torque_abs}
    ChVectorDynamic<> mF(loadable->GetNumFieldCoords());
    mF(0) = abs_force.x();
    mF(1) = abs_force.y();
    mF(2) = abs_force.z();
    mF(3) = 0;
    mF(4) = 0;
    mF(5) = 0;

    // Compute Q = N(u,v,w)'*F
    double detJ;  // not used
    mbody->ComputeNF(abs_point.x(), abs_point.y(), abs_point.z(), load_Q, detJ, mF, state_x, state_w);
}

void ChLoadBodyForce::Update(double time) {
    if (!std::dynamic_pointer_cast<ChBody>(this->loadable)->Variables().IsActive())
        return;

    m_modulation->Update(time);
    m_scale = m_modulation->GetVal(time);
    ChLoadCustom::Update(time);
}

void ChLoadBodyForce::SetForce(const ChVector3d& force, bool is_local) {
    m_force = force;
    m_local_force = is_local;
}

void ChLoadBodyForce::SetApplicationPoint(const ChVector3d& point, const bool is_local) {
    m_point = point;
    m_local_point = is_local;
}

ChVector3d ChLoadBodyForce::GetForce() const {
    return m_force * m_scale;
}

// -----------------------------------------------------------------------------
// ChLoadBodyTorque
// -----------------------------------------------------------------------------

ChLoadBodyTorque::ChLoadBodyTorque(std::shared_ptr<ChBody> body, const ChVector3d& torque, bool local_torque)
    : ChLoadCustom(body), m_torque(torque), m_local_torque(local_torque), m_scale(1) {
    m_modulation = chrono_types::make_shared<ChFunctionConst>(1.0);
}

void ChLoadBodyTorque::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mbody = std::dynamic_pointer_cast<ChBody>(this->loadable);
    if (!mbody->Variables().IsActive())
        return;

    ChVector3d abs_torque;
    if (m_local_torque)
        abs_torque = mbody->TransformDirectionLocalToParent(m_torque);
    else
        abs_torque = m_torque;

    abs_torque *= m_scale;

    // ChBody assumes F={force_abs, torque_abs}
    ChVectorDynamic<> mF(loadable->GetNumFieldCoords());
    mF(0) = 0;
    mF(1) = 0;
    mF(2) = 0;
    mF(3) = abs_torque.x();
    mF(4) = abs_torque.y();
    mF(5) = abs_torque.z();

    // Compute Q = N(u,v,w)'*F
    double detJ;  // not used
    mbody->ComputeNF(0, 0, 0, load_Q, detJ, mF, state_x, state_w);
}

void ChLoadBodyTorque::Update(double time) {
    if (!std::dynamic_pointer_cast<ChBody>(this->loadable)->Variables().IsActive())
        return;

    m_modulation->Update(time);
    m_scale = m_modulation->GetVal(time);
    ChLoadCustom::Update(time);
}

void ChLoadBodyTorque::SetTorque(const ChVector3d& torque, bool is_local) {
    m_torque = torque;
    m_local_torque = is_local;
}

ChVector3d ChLoadBodyTorque::GetTorque() const {
    return m_torque * m_scale;
}

// -----------------------------------------------------------------------------
// ChLoadBodyInertia
// -----------------------------------------------------------------------------

// For testing and optimizations.
// Default true. Can be disabled globally, for testing or optimization
bool ChLoadBodyInertia::use_inertial_damping_matrix_R = true;
bool ChLoadBodyInertia::use_inertial_stiffness_matrix_K = true;
bool ChLoadBodyInertia::use_gyroscopic_torque = true;

ChLoadBodyInertia::ChLoadBodyInertia(std::shared_ptr<ChBody> body,
                                     const ChVector3d& offset,
                                     const double m,
                                     const ChVector3d& IXX,
                                     const ChVector3d& IXY)
    : ChLoadCustom(body), c_m(offset), mass(m) {
    SetInertiaXX(IXX);
    SetInertiaXY(IXY);
}

// The inertia tensor functions

void ChLoadBodyInertia::SetInertia(const ChMatrix33<>& newXInertia) {
    I = newXInertia;
}

void ChLoadBodyInertia::SetInertiaXX(const ChVector3d& iner) {
    I(0, 0) = iner.x();
    I(1, 1) = iner.y();
    I(2, 2) = iner.z();
}

void ChLoadBodyInertia::SetInertiaXY(const ChVector3d& iner) {
    I(0, 1) = iner.x();
    I(0, 2) = iner.y();
    I(1, 2) = iner.z();
    I(1, 0) = iner.x();
    I(2, 0) = iner.y();
    I(2, 1) = iner.z();
}

ChVector3d ChLoadBodyInertia::GetInertiaXX() const {
    ChVector3d iner;
    iner.x() = I(0, 0);
    iner.y() = I(1, 1);
    iner.z() = I(2, 2);
    return iner;
}

ChVector3d ChLoadBodyInertia::GetInertiaXY() const {
    ChVector3d iner;
    iner.x() = I(0, 1);
    iner.y() = I(0, 2);
    iner.z() = I(1, 2);
    return iner;
}

void ChLoadBodyInertia::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mbody = std::dynamic_pointer_cast<ChBody>(this->loadable);
    if (!mbody->Variables().IsActive())
        return;

    // fetch speeds/pos/accel as 3d vectors for convenience
    ChVector3d v_x = state_w->segment(0, 3);  // abs.
    ChVector3d v_w = state_w->segment(3, 3);  // local

    /* // NO ACCELERATION PROPORTIONAL TERM ADDED HERE! Can use LoadIntLoadResidual_Mv if needed.
    ChVector3d Ma_x = this->mass * (a_x + chrono::Vcross(a_w, this->c_m));
    ChVector3d Ma_w = this->mass * chrono::Vcross(this->c_m, a_x) + this->I * a_w;
    */

    // Terms of inertial quadratic type (centrifugal, gyroscopic)
    ChVector3d quadratic_x;
    ChVector3d quadratic_w;
    quadratic_x = this->mass * chrono::Vcross(v_w, chrono::Vcross(v_w, this->c_m));  // centrifugal: m*(w X w X c_m)
    if (this->use_gyroscopic_torque)
        quadratic_w = chrono::Vcross(v_w, this->I * v_w);  // gyroscopical: w X J*w
    else
        quadratic_w = VNULL;

    load_Q.segment(0, 3) = -(quadratic_x).eigen();  // sign: negative, as Q goes in RHS
    load_Q.segment(3, 3) = -(quadratic_w).eigen();  // sign: negative, as Q goes in RHS
}

void ChLoadBodyInertia::ComputeJacobian(ChState* state_x, ChStateDelta* state_w) {
    // fetch speeds as 3d vectors for convenience
    ChVector3d v_x = state_w->segment(0, 3);  // abs.
    ChVector3d v_w = state_w->segment(3, 3);  // local
    // (note: accelerations should be fetched from a "state_acc" like we did for speeds with state_w,
    // but acc. are not available in ComputeQ inputs... maybe in future we should change the ChLoad
    // class to support also this. For this special case, it works also with the following trick, but
    // would fail the default numerical differentiation in ComputeJacobian() for the M=dQ/da matrix, so
    // we override ComputeJacobian and provide the analytical jacobians)
    auto mbody = std::dynamic_pointer_cast<ChBody>(this->loadable);
    ChVector3d a_x = mbody->GetRotMat().transpose() * mbody->GetPosDt2();  // local
    ChVector3d a_w = mbody->GetAngAccLocal();                              // local

    ChStarMatrix33<> wtilde(v_w);  // [w~]
    ChStarMatrix33<> ctilde(c_m);  // [c~]

    // Analytic expression of inertial load jacobians.
    // Note signs: positive as they go in LHS.

    // M mass matrix terms (6x6, split in four 3x3 blocks for convenience)
    m_jacobians->M.setZero();
    m_jacobians->M.block(0, 0, 3, 3).diagonal().setConstant(this->mass);
    m_jacobians->M.block(0, 3, 3, 3) = this->mass * chrono::ChStarMatrix33<>(-this->c_m);
    m_jacobians->M.block(3, 0, 3, 3) = this->mass * chrono::ChStarMatrix33<>(this->c_m);
    m_jacobians->M.block(3, 3, 3, 3) = this->I;

    // R gyroscopic damping matrix terms (6x6, split in 3x3 blocks for convenience)
    m_jacobians->R.setZero();
    if (this->use_inertial_damping_matrix_R) {
        //  Ri = [0, - m*[w~][c~] - m*[([w~]*c)~]  ; 0 , [w~][I] - [([I]*w)~]  ]
        m_jacobians->R.block(0, 3, 3, 3) = -this->mass * (wtilde * ctilde + ChStarMatrix33<>(wtilde * c_m));
        m_jacobians->R.block(3, 3, 3, 3) = wtilde * I - ChStarMatrix33<>(I * v_w);
    }

    // K inertial stiffness matrix terms (6x6, split in 3x3 blocks for convenience)
    m_jacobians->K.setZero();
    if (this->use_inertial_stiffness_matrix_K) {
        ChStarMatrix33<> atilde(a_w);  // [a~]
        // Ki_al = [0, -m*[([a~]c)~] -m*[([w~][w~]c)~] ; 0, m*[c~][xpp~] ]
        m_jacobians->K.block(0, 3, 3, 3) =
            -this->mass * ChStarMatrix33<>(atilde * c_m) - this->mass * ChStarMatrix33<>(wtilde * (wtilde * c_m));
        m_jacobians->K.block(3, 3, 3, 3) = this->mass * ctilde * ChStarMatrix33<>(a_x);
    }
}

// The default base implementation in ChLoadCustom could suffice, but here reimplement it in sake of higher speed
// because we can exploiti the sparsity of the formulas.
void ChLoadBodyInertia::LoadIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {
    if (!m_jacobians)
        return;

    if (!loadable->IsSubBlockActive(0))
        return;

    // fetch w as a contiguous vector
    ChVector3d a_x = w.segment(loadable->GetSubBlockOffset(0), 3);
    ChVector3d a_w = w.segment(loadable->GetSubBlockOffset(0) + 3, 3);

    // R+=c*M*a
    R.segment(loadable->GetSubBlockOffset(0), 3) += c * (this->mass * (a_x + chrono::Vcross(a_w, this->c_m))).eigen();
    R.segment(loadable->GetSubBlockOffset(0) + 3, 3) +=
        c * (this->mass * chrono::Vcross(this->c_m, a_x) + this->I * a_w).eigen();
}

// -----------------------------------------------------------------------------
// ChLoadBodyBody
// -----------------------------------------------------------------------------

ChLoadBodyBody::ChLoadBodyBody(std::shared_ptr<ChBody> bodyA,
                               std::shared_ptr<ChBody> bodyB,
                               const ChFrame<>& abs_application)
    : ChLoadCustomMultiple(bodyA, bodyB) {
    loc_application_A = bodyA->ChFrame::TransformParentToLocal(abs_application);
    loc_application_B = bodyB->ChFrame::TransformParentToLocal(abs_application);
}

void ChLoadBodyBody::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
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

    ComputeBodyBodyForceTorque(rel_AB, locB_force, locB_torque);

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

std::shared_ptr<ChBody> ChLoadBodyBody::GetBodyA() const {
    return std::dynamic_pointer_cast<ChBody>(this->loadables[0]);
}

std::shared_ptr<ChBody> ChLoadBodyBody::GetBodyB() const {
    return std::dynamic_pointer_cast<ChBody>(this->loadables[1]);
}

// -----------------------------------------------------------------------------
// ChLoadBodyBodyTorque
// -----------------------------------------------------------------------------

ChLoadBodyBodyTorque::ChLoadBodyBodyTorque(std::shared_ptr<ChBody> bodyA,
                                           std::shared_ptr<ChBody> bodyB,
                                           const ChVector3d torque,
                                           bool local_torque)
    : ChLoadBodyBody(bodyA, bodyB, ChFrame<>()), m_torque(torque), m_local_torque(local_torque), m_scale(1) {
    m_modulation = chrono_types::make_shared<ChFunctionConst>(1.0);
}

void ChLoadBodyBodyTorque::ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
                                                      ChVector3d& loc_force,
                                                      ChVector3d& loc_torque) {
    loc_force = VNULL;

    if (m_local_torque)
        loc_torque = m_torque;
    else {
        auto bodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);
        loc_torque = bodyB->TransformDirectionParentToLocal(m_torque);
    }

    loc_torque *= m_scale;
}

void ChLoadBodyBodyTorque::Update(double time) {
    m_modulation->Update(time);
    m_scale = m_modulation->GetVal(time);
    ChLoadCustomMultiple::Update(time);
}

// -----------------------------------------------------------------------------
// ChLoadBodyBodyBushingSpherical
// -----------------------------------------------------------------------------

ChLoadBodyBodyBushingSpherical::ChLoadBodyBodyBushingSpherical(std::shared_ptr<ChBody> bodyA,
                                                               std::shared_ptr<ChBody> bodyB,
                                                               const ChFrame<>& abs_application,
                                                               const ChVector3d& stiffness_coefs,
                                                               const ChVector3d& damping_coefs)
    : ChLoadBodyBody(bodyA, bodyB, abs_application), stiffness(stiffness_coefs), damping(damping_coefs) {}

void ChLoadBodyBodyBushingSpherical::ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
                                                                ChVector3d& loc_force,
                                                                ChVector3d& loc_torque) {
    loc_force = rel_AB.GetPos() * stiffness     // element-wise product!
                + rel_AB.GetPosDt() * damping;  // element-wise product!
    loc_torque = VNULL;
}

// -----------------------------------------------------------------------------
// ChLoadBodyBodyBushingPlastic
// -----------------------------------------------------------------------------

ChLoadBodyBodyBushingPlastic::ChLoadBodyBodyBushingPlastic(std::shared_ptr<ChBody> bodyA,
                                                           std::shared_ptr<ChBody> bodyB,
                                                           const ChFrame<>& abs_application,
                                                           const ChVector3d& stiffness,
                                                           const ChVector3d& damping,
                                                           const ChVector3d& plastic_yield)
    : ChLoadBodyBodyBushingSpherical(bodyA, bodyB, abs_application, stiffness, damping),
      yield(plastic_yield),
      plastic_def(VNULL) {}

void ChLoadBodyBodyBushingPlastic::ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
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

ChLoadBodyBodyBushingMate::ChLoadBodyBodyBushingMate(std::shared_ptr<ChBody> bodyA,
                                                     std::shared_ptr<ChBody> bodyB,
                                                     const ChFrame<>& abs_application,
                                                     const ChVector3d& stiffness,
                                                     const ChVector3d& damping,
                                                     const ChVector3d& rotstiffness,
                                                     const ChVector3d& rotdamping)
    : ChLoadBodyBodyBushingSpherical(bodyA, bodyB, abs_application, stiffness, damping),
      rot_stiffness(rotstiffness),
      rot_damping(rotdamping) {}

void ChLoadBodyBodyBushingMate::ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
                                                           ChVector3d& loc_force,
                                                           ChVector3d& loc_torque) {
    // inherit parent to compute loc_force = ...
    ChLoadBodyBodyBushingSpherical::ComputeBodyBodyForceTorque(rel_AB, loc_force, loc_torque);

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
// ChLoadBodyBodyBushingGeneric
// -----------------------------------------------------------------------------

ChLoadBodyBodyBushingGeneric::ChLoadBodyBodyBushingGeneric(std::shared_ptr<ChBody> bodyA,
                                                           std::shared_ptr<ChBody> bodyB,
                                                           const ChFrame<>& abs_application,
                                                           const ChMatrix66d& stiffness66,
                                                           const ChMatrix66d& damping66)
    : ChLoadBodyBody(bodyA, bodyB, abs_application), stiffness(stiffness66), damping(damping66) {}

void ChLoadBodyBodyBushingGeneric::ComputeBodyBodyForceTorque(const ChFrameMoving<>& rel_AB,
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

}  // end namespace chrono
