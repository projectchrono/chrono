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

#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkNodeSlopeFrame)

ChLinkNodeSlopeFrame::ChLinkNodeSlopeFrame() : m_react(VNULL), m_csys(CSYSNORM) {}

ChLinkNodeSlopeFrame::ChLinkNodeSlopeFrame(const ChLinkNodeSlopeFrame& other) : ChLinkBase(other) {
    m_react = other.m_react;
    m_csys = other.m_csys;
}

ChFrame<> ChLinkNodeSlopeFrame::GetFrameNodeAbs() const {
    return ChFrame<>(m_node->GetPos(), m_csys.rot >> m_body->GetRot());
}

void ChLinkNodeSlopeFrame::SetDirectionInBodyCoords(const ChVector3d& dir_loc) {
    assert(m_body);
    ChMatrix33<> rot;
    rot.SetFromAxisX(dir_loc);
    m_csys.rot = rot.GetQuaternion();
    m_csys.pos = VNULL;
}

void ChLinkNodeSlopeFrame::SetDirectionInAbsoluteCoords(const ChVector3d& dir_abs) {
    assert(m_body);
    ChVector3d dir_loc = m_body->TransformDirectionParentToLocal(dir_abs);
    SetDirectionInBodyCoords(dir_loc);
}

int ChLinkNodeSlopeFrame::Initialize(std::shared_ptr<ChNodeFEAxyzD> node,
                                     std::shared_ptr<ChBodyFrame> body,
                                     ChVector3d* dir) {
    assert(node && body);

    m_body = body;
    m_node = node;

    constraint1.SetVariables(&(node->VariablesSlope1()), &(body->Variables()));
    constraint2.SetVariables(&(node->VariablesSlope1()), &(body->Variables()));

    ChVector3d dir_abs = dir ? *dir : node->GetSlope1();
    SetDirectionInAbsoluteCoords(dir_abs);

    return true;
}

void ChLinkNodeSlopeFrame::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // ...
}

ChVectorDynamic<> ChLinkNodeSlopeFrame::GetConstraintViolation() const {
    ChMatrix33<> Arw = m_csys.rot >> m_body->GetRot();
    ChVector3d res = Arw.transpose() * m_node->GetSlope1();
    ChVectorN<double, 2> C;
    C(0) = res.y();
    C(1) = res.z();
    return C;
}

ChVector3d ChLinkNodeSlopeFrame::GetReactionOnBody() const {
    // Rotation matrices
    ChMatrix33<> A(m_body->GetRot());
    ChMatrix33<> C(m_csys.rot);

    // (A^T*d)  and  ~(A^T*d)
    ChVector3d z = A.transpose() * m_node->GetSlope1();
    ChStarMatrix33<> ztilde(z);

    // Constraint Jacobians  PhiQ = C^T * ~(A^T*d)
    ChMatrix33<> PhiQ = C.transpose() * ztilde;

    // Reaction torque  T = C^T * PhiQ^T * lambda
    // Note that lambda = [0, l1, l2]
    ChVector3d trq = C.transpose() * (PhiQ.transpose() * m_react);

    return trq;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkNodeSlopeFrame::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = m_react.y();
    L(off_L + 1) = m_react.z();
}

void ChLinkNodeSlopeFrame::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    m_react.y() = L(off_L + 0);
    m_react.z() = L(off_L + 1);
}

void ChLinkNodeSlopeFrame::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                               ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                               const ChVectorDynamic<>& L,  // the L vector
                                               const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    constraint2.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
}

void ChLinkNodeSlopeFrame::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                               ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                               const double c,            // a scaling factor
                                               bool do_clamp,             // apply clamping to c*C?
                                               double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    ChMatrix33<> Arw = m_csys.rot >> m_body->GetRot();
    ChVector3d cres = c * (Arw.transpose() * m_node->GetSlope1());

    if (do_clamp) {
        cres.y() = std::min(std::max(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = std::min(std::max(cres.z(), -recovery_clamp), recovery_clamp);
    }

    Qc(off_L + 0) += cres.y();
    Qc(off_L + 1) += cres.z();
}

void ChLinkNodeSlopeFrame::IntToDescriptor(const unsigned int off_v,
                                           const ChStateDelta& v,
                                           const ChVectorDynamic<>& R,
                                           const unsigned int off_L,
                                           const ChVectorDynamic<>& L,
                                           const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    constraint1.SetLagrangeMultiplier(L(off_L + 0));
    constraint2.SetLagrangeMultiplier(L(off_L + 1));

    constraint1.SetRightHandSide(Qc(off_L + 0));
    constraint2.SetRightHandSide(Qc(off_L + 1));
}

void ChLinkNodeSlopeFrame::IntFromDescriptor(const unsigned int off_v,
                                             ChStateDelta& v,
                                             const unsigned int off_L,
                                             ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = constraint1.GetLagrangeMultiplier();
    L(off_L + 1) = constraint2.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkNodeSlopeFrame::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&constraint1);
    descriptor.InsertConstraint(&constraint2);
}

void ChLinkNodeSlopeFrame::ConstraintsBiReset() {
    constraint1.SetRightHandSide(0.);
    constraint2.SetRightHandSide(0.);
}

void ChLinkNodeSlopeFrame::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    ChMatrix33<> Arw = m_csys.rot >> m_body->GetRot();
    ChVector3d res = Arw.transpose() * m_node->GetSlope1();

    constraint1.SetRightHandSide(constraint1.GetRightHandSide() + factor * res.y());
    constraint2.SetRightHandSide(constraint2.GetRightHandSide() + factor * res.z());
}

void ChLinkNodeSlopeFrame::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkNodeSlopeFrame::LoadConstraintJacobians() {
    // compute jacobians
    ChMatrix33<> Aow(m_body->GetRot());
    ChMatrix33<> Aro(m_csys.rot);
    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector3d Zo = Aow.transpose() * m_node->GetSlope1();

    ChStarMatrix33<> ztilde(Zo);

    ChMatrix33<> Jrb = Aro.transpose() * ztilde;

    ChMatrix33<> Jra = Arw.transpose();

    constraint1.Get_Cq_a().segment(0, 3) = Jra.row(1);
    constraint2.Get_Cq_a().segment(0, 3) = Jra.row(2);

    constraint1.Get_Cq_b().segment(3, 3) = Jrb.row(1);
    constraint2.Get_Cq_b().segment(3, 3) = Jrb.row(2);
}

void ChLinkNodeSlopeFrame::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    m_react.y() = constraint1.GetLagrangeMultiplier() * factor;
    m_react.z() = constraint2.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkNodeSlopeFrame::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkNodeSlopeFrame::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
