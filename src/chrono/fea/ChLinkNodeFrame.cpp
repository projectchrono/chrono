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
#include "chrono/fea/ChLinkNodeFrame.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkNodeFrameGeneric)

ChLinkNodeFrameGeneric::ChLinkNodeFrameGeneric(bool mc_x, bool mc_y, bool mc_z) : m_react(VNULL), m_csys(CSYSNORM) {
    c_x = mc_x;
    c_y = mc_y;
    c_z = mc_z;
}

ChLinkNodeFrameGeneric::ChLinkNodeFrameGeneric(const ChLinkNodeFrameGeneric& other) : ChLinkBase(other) {
    m_csys = other.m_csys;
    m_react = other.m_react;
    c_x = other.c_x;
    c_y = other.c_y;
    c_z = other.c_z;
}

ChFrame<> ChLinkNodeFrameGeneric::GetFrameNodeAbs() const {
    return ChFrame<>(m_csys >> *m_body);
}

int ChLinkNodeFrameGeneric::Initialize(std::shared_ptr<ChNodeFEAxyz> node,  ///< xyz node (point) to join
                                       std::shared_ptr<ChBodyFrame> body,   ///< body (frame) to join
                                       chrono::ChCoordsys<> csys_abs) {
    assert(node && body);

    m_body = body;
    m_node = node;

    m_constraint1.SetVariables(&(node->Variables()), &(body->Variables()));
    m_constraint2.SetVariables(&(node->Variables()), &(body->Variables()));
    m_constraint3.SetVariables(&(node->Variables()), &(body->Variables()));

    m_csys = m_body->GetCoordsys().TransformParentToLocal(csys_abs);

    return true;
}

int ChLinkNodeFrameGeneric::Initialize(std::shared_ptr<ChNodeFEAxyz> node,
                                       std::shared_ptr<ChBodyFrame> body,
                                       ChVector3d* pos) {
    ChVector3d pos_abs = pos ? *pos : node->GetPos();

    return this->Initialize(node, body, ChCoordsys<>(pos_abs));
}

void ChLinkNodeFrameGeneric::SetConstrainedCoords(bool mc_x, bool mc_y, bool mc_z) {
    c_x = mc_x;
    c_y = mc_y;
    c_z = mc_z;
}

void ChLinkNodeFrameGeneric::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    // ...
}

ChVectorDynamic<> ChLinkNodeFrameGeneric::GetConstraintViolation() const {
    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());
    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
    ChVectorN<double, 3> C;
    C(0) = res.x();
    C(1) = res.y();
    C(2) = res.z();
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkNodeFrameGeneric::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    int nc = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        L(off_L + nc) = m_react.x();
        nc++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        L(off_L + nc) = m_react.y();
        nc++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        L(off_L + nc) = m_react.z();
        // nc++;
    }
}

void ChLinkNodeFrameGeneric::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    int nc = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        m_react.x() = L(off_L + nc);
        nc++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        m_react.y() = L(off_L + nc);
        nc++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        m_react.z() = L(off_L + nc);
        // nc++;
    }
}

void ChLinkNodeFrameGeneric::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                                 ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                                 const ChVectorDynamic<>& L,  // the L vector
                                                 const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    int cnt = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        m_constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + cnt) * c);
        cnt++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        m_constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + cnt) * c);
        cnt++;
    }
    if (c_y && this->m_constraint3.IsActive()) {
        m_constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + cnt) * c);
        // cnt++;
    }
}

void ChLinkNodeFrameGeneric::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                                 ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                                 const double c,            // a scaling factor
                                                 bool do_clamp,             // apply clamping to c*C?
                                                 double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
    ChVector3d cres = res * c;

    if (do_clamp) {
        cres.x() = std::min(std::max(cres.x(), -recovery_clamp), recovery_clamp);
        cres.y() = std::min(std::max(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = std::min(std::max(cres.z(), -recovery_clamp), recovery_clamp);
    }

    int cnt = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        Qc(off_L + cnt) += cres.x();
        cnt++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        Qc(off_L + cnt) += cres.y();
        cnt++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        Qc(off_L + cnt) += cres.z();
        // cnt++;
    }
}

void ChLinkNodeFrameGeneric::IntToDescriptor(const unsigned int off_v,
                                             const ChStateDelta& v,
                                             const ChVectorDynamic<>& R,
                                             const unsigned int off_L,
                                             const ChVectorDynamic<>& L,
                                             const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    int cnt = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        m_constraint1.SetLagrangeMultiplier(L(off_L + cnt));
        m_constraint1.SetRightHandSide(Qc(off_L + cnt));
        cnt++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        m_constraint2.SetLagrangeMultiplier(L(off_L + cnt));
        m_constraint2.SetRightHandSide(Qc(off_L + cnt));
        cnt++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        m_constraint3.SetLagrangeMultiplier(L(off_L + cnt));
        m_constraint3.SetRightHandSide(Qc(off_L + cnt));
        // cnt++;
    }
}

void ChLinkNodeFrameGeneric::IntFromDescriptor(const unsigned int off_v,
                                               ChStateDelta& v,
                                               const unsigned int off_L,
                                               ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    int cnt = 0;
    if (c_x && this->m_constraint1.IsActive()) {
        L(off_L + cnt) = m_constraint1.GetLagrangeMultiplier();
        cnt++;
    }
    if (c_y && this->m_constraint2.IsActive()) {
        L(off_L + cnt) = m_constraint2.GetLagrangeMultiplier();
        cnt++;
    }
    if (c_z && this->m_constraint3.IsActive()) {
        L(off_L + cnt) = m_constraint3.GetLagrangeMultiplier();
        // cnt++;
    }
}

// SOLVER INTERFACES

void ChLinkNodeFrameGeneric::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    if (c_x && this->m_constraint1.IsActive())
        descriptor.InsertConstraint(&m_constraint1);

    if (c_y && this->m_constraint2.IsActive())
        descriptor.InsertConstraint(&m_constraint2);

    if (c_z && this->m_constraint3.IsActive())
        descriptor.InsertConstraint(&m_constraint3);
}

void ChLinkNodeFrameGeneric::ConstraintsBiReset() {
    m_constraint1.SetRightHandSide(0.);
    m_constraint2.SetRightHandSide(0.);
    m_constraint3.SetRightHandSide(0.);
}

void ChLinkNodeFrameGeneric::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    if (!m_node)
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));

    if (c_x && this->m_constraint1.IsActive()) {
        m_constraint1.SetRightHandSide(m_constraint1.GetRightHandSide() + factor * res.x());
    }
    if (c_y && this->m_constraint2.IsActive()) {
        m_constraint2.SetRightHandSide(m_constraint2.GetRightHandSide() + factor * res.y());
    }
    if (c_z && this->m_constraint3.IsActive()) {
        m_constraint3.SetRightHandSide(m_constraint3.GetRightHandSide() + factor * res.z());
    }
}

void ChLinkNodeFrameGeneric::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkNodeFrameGeneric::LoadConstraintJacobians() {
    // compute jacobians
    ChMatrix33<> Aro(m_csys.rot);
    ChMatrix33<> Aow(m_body->GetRot());
    ChMatrix33<> Arw = Aow * Aro;

    ChMatrix33<> Jxn = Arw.transpose();

    ChMatrix33<> Jxb = -Arw.transpose();

    ChStarMatrix33<> atilde(Aow.transpose() * (m_node->GetPos() - m_body->GetPos()));

    ChMatrix33<> Jrb = Aro.transpose() * atilde;

    if (c_x) {
        m_constraint1.Get_Cq_a().segment(0, 3) = Jxn.row(0);
        m_constraint1.Get_Cq_b().segment(0, 3) = Jxb.row(0);
        m_constraint1.Get_Cq_b().segment(3, 3) = Jrb.row(0);
    }
    if (c_y) {
        m_constraint2.Get_Cq_a().segment(0, 3) = Jxn.row(1);
        m_constraint2.Get_Cq_b().segment(0, 3) = Jxb.row(1);
        m_constraint2.Get_Cq_b().segment(3, 3) = Jrb.row(1);
    }
    if (c_z) {
        m_constraint3.Get_Cq_a().segment(0, 3) = Jxn.row(2);
        m_constraint3.Get_Cq_b().segment(0, 3) = Jxb.row(2);
        m_constraint3.Get_Cq_b().segment(3, 3) = Jrb.row(2);
    }
}

void ChLinkNodeFrameGeneric::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    m_react.x() = m_constraint1.GetLagrangeMultiplier() * factor;
    m_react.y() = m_constraint2.GetLagrangeMultiplier() * factor;
    m_react.z() = m_constraint3.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkNodeFrameGeneric::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkNodeFrameGeneric::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkNodeFrame)

ChLinkNodeFrame::ChLinkNodeFrame() : m_react(VNULL), m_csys(CSYSNORM) {}

ChLinkNodeFrame::ChLinkNodeFrame(const ChLinkNodeFrame& other) : ChLinkBase(other) {
    m_csys = other.m_csys;
    m_react = other.m_react;
}

ChFrame<> ChLinkNodeFrame::GetFrameNodeAbs() const {
    return ChFrame<>(m_csys >> *m_body);
}

int ChLinkNodeFrame::Initialize(std::shared_ptr<ChNodeFEAxyz> node,
                                std::shared_ptr<ChBodyFrame> body,
                                const ChVector3d* pos) {
    assert(node && body);

    m_body = body;
    m_node = node;

    m_constraint1.SetVariables(&(node->Variables()), &(body->Variables()));
    m_constraint2.SetVariables(&(node->Variables()), &(body->Variables()));
    m_constraint3.SetVariables(&(node->Variables()), &(body->Variables()));

    ChVector3d pos_abs = pos ? *pos : node->GetPos();
    SetAttachPositionInAbsoluteCoords(pos_abs);

    return true;
}

void ChLinkNodeFrame::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    // ...
}

ChVectorDynamic<> ChLinkNodeFrame::GetConstraintViolation() const {
    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());
    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
    ChVectorN<double, 3> C;
    C(0) = res.x();
    C(1) = res.y();
    C(2) = res.z();
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkNodeFrame::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = m_react.x();
    L(off_L + 1) = m_react.y();
    L(off_L + 2) = m_react.z();
}

void ChLinkNodeFrame::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    m_react.x() = L(off_L + 0);
    m_react.y() = L(off_L + 1);
    m_react.z() = L(off_L + 2);
}

void ChLinkNodeFrame::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                          ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                          const ChVectorDynamic<>& L,  // the L vector
                                          const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    m_constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    m_constraint2.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
    m_constraint3.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
}

void ChLinkNodeFrame::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                          ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                          const double c,            // a scaling factor
                                          bool do_clamp,             // apply clamping to c*C?
                                          double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));
    ChVector3d cres = res * c;

    if (do_clamp) {
        cres.x() = std::min(std::max(cres.x(), -recovery_clamp), recovery_clamp);
        cres.y() = std::min(std::max(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = std::min(std::max(cres.z(), -recovery_clamp), recovery_clamp);
    }
    Qc(off_L + 0) += cres.x();
    Qc(off_L + 1) += cres.y();
    Qc(off_L + 2) += cres.z();
}

void ChLinkNodeFrame::IntToDescriptor(const unsigned int off_v,
                                      const ChStateDelta& v,
                                      const ChVectorDynamic<>& R,
                                      const unsigned int off_L,
                                      const ChVectorDynamic<>& L,
                                      const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_constraint1.SetLagrangeMultiplier(L(off_L + 0));
    m_constraint2.SetLagrangeMultiplier(L(off_L + 1));
    m_constraint3.SetLagrangeMultiplier(L(off_L + 2));

    m_constraint1.SetRightHandSide(Qc(off_L + 0));
    m_constraint2.SetRightHandSide(Qc(off_L + 1));
    m_constraint3.SetRightHandSide(Qc(off_L + 2));
}

void ChLinkNodeFrame::IntFromDescriptor(const unsigned int off_v,
                                        ChStateDelta& v,
                                        const unsigned int off_L,
                                        ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_constraint1.GetLagrangeMultiplier();
    L(off_L + 1) = m_constraint2.GetLagrangeMultiplier();
    L(off_L + 2) = m_constraint3.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkNodeFrame::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&m_constraint1);
    descriptor.InsertConstraint(&m_constraint2);
    descriptor.InsertConstraint(&m_constraint3);
}

void ChLinkNodeFrame::ConstraintsBiReset() {
    m_constraint1.SetRightHandSide(0.);
    m_constraint2.SetRightHandSide(0.);
    m_constraint3.SetRightHandSide(0.);
}

void ChLinkNodeFrame::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    if (!m_node)
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector3d res = Arw.transpose() * (m_node->GetPos() - m_body->TransformPointLocalToParent(m_csys.pos));

    m_constraint1.SetRightHandSide(m_constraint1.GetRightHandSide() + factor * res.x());
    m_constraint2.SetRightHandSide(m_constraint2.GetRightHandSide() + factor * res.y());
    m_constraint3.SetRightHandSide(m_constraint3.GetRightHandSide() + factor * res.z());
}

void ChLinkNodeFrame::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkNodeFrame::LoadConstraintJacobians() {
    // compute jacobians
    ChMatrix33<> Aro(m_csys.rot);
    ChMatrix33<> Aow(m_body->GetRot());
    ChMatrix33<> Arw = Aow * Aro;

    ChMatrix33<> Jxn = Arw.transpose();

    ChMatrix33<> Jxb = -Arw.transpose();

    ChStarMatrix33<> atilde(Aow.transpose() * (m_node->GetPos() - m_body->GetPos()));
    ChMatrix33<> Jrb = Aro.transpose() * atilde;

    m_constraint1.Get_Cq_a().segment(0, 3) = Jxn.row(0);
    m_constraint2.Get_Cq_a().segment(0, 3) = Jxn.row(1);
    m_constraint3.Get_Cq_a().segment(0, 3) = Jxn.row(2);

    m_constraint1.Get_Cq_b().segment(0, 3) = Jxb.row(0);
    m_constraint2.Get_Cq_b().segment(0, 3) = Jxb.row(1);
    m_constraint3.Get_Cq_b().segment(0, 3) = Jxb.row(2);

    m_constraint1.Get_Cq_b().segment(3, 3) = Jrb.row(0);
    m_constraint2.Get_Cq_b().segment(3, 3) = Jrb.row(1);
    m_constraint3.Get_Cq_b().segment(3, 3) = Jrb.row(2);
}

void ChLinkNodeFrame::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    m_react.x() = m_constraint1.GetLagrangeMultiplier() * factor;
    m_react.y() = m_constraint2.GetLagrangeMultiplier() * factor;
    m_react.z() = m_constraint3.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkNodeFrame::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkNodeFrame::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
